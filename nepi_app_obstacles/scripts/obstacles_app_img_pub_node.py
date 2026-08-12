#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi applications (nepi_apps) repo
# (see https://https://github.com/nepi-engine/nepi_apps)
#
# License: nepi applications are licensed under the "Numurus Software License",
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com
#


import os
import copy
import numpy as np
import math
import cv2
import threading


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_img

from sensor_msgs.msg import Image

from nepi_interfaces.msg import ImageStatus
from nepi_interfaces.msg import ProcessStatus

from nepi_app_obstacles.msg import Obstacles, ObstaclesDepthMap, ObstaclesStatus

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import SaveDataIF
from nepi_api.data_if import ColorImageIF


WATCHDOG_DELAY = 60
WATCHDOG_TIMEOUT = 3

# How much source frame history the aligned-image lookup can search. Two seconds
# is well past any observed process latency, and the count cap keeps a fast
# source from holding more than a handful of frames. Raw ROS Image msgs are
# buffered undecoded, so a frame that never gets published costs only its bytes.
MAX_IMG_BUFFER_SEC = 2.0
MAX_IMG_BUFFER_LEN = 10

# Overlay colours, BGR
OBSTACLE_BOX_COLOR = (0, 255, 0)

# The ground overlay is painted one flat colour over the whole ground segment
# rather than colourized by range -- what the operator wants from it is where the
# drivable surface is, and a solid fill reads that at a glance. The obstacles
# overlay stays range-colourized, because distance to an obstacle is the point of
# it. Set to None to colourize instead.
GROUND_OVERLAY_COLOR = (0, 255, 0)
OBSTACLES_OVERLAY_COLOR = None

# Fallbacks for the transparency the parent reports on its status message, used
# only until the first status arrives. 0.0 fully opaque through 1.0 invisible.
DEFAULT_GROUND_TRANSPARENCY = 0.6
DEFAULT_OBSTACLES_TRANSPARENCY = 0.4

# Colour painted over every non-member pixel of a segmentation render, BGR.
# The platform colorizer folds NaN onto the max-range colour, which is the same
# dark blue a genuine far return gets -- so "not in this segment" would read as
# "at the far end of the range". Black is outside the jet colormap entirely, so
# the segment boundary is unambiguous.
SEGMENT_NONE_COLOR = (0, 0, 0)


class ObstaclesImgPub:

    OBSTACLES_IMG_DATA_PRODUCT = 'obstacles_image'
    GROUND_MAP_IMG_DATA_PRODUCT = 'ground_depth_map_image'
    OBSTACLES_MAP_IMG_DATA_PRODUCT = 'obstacles_depth_map_image'

    # The two depth map segmentation image products. Each entry drives one raw
    # publisher, one ColorImageIF and one img_node_dict key pair, created and
    # purged with its source exactly like obstacles_image. Named <range
    # product>_image after the platform convention DepthMapIF sets: the raw
    # 32FC1 range data is <name>, its colourized viewable render is <name>_image.
    SEGMENT_IMG_PRODUCTS = [
        {'data_product': GROUND_MAP_IMG_DATA_PRODUCT,
         'map_key': 'depth_map_ground',
         'if_key': 'ground_img_if',
         'pub_key': 'ground_img_pub'},
        {'data_product': OBSTACLES_MAP_IMG_DATA_PRODUCT,
         'map_key': 'depth_map_obstacles',
         'if_key': 'obstacles_map_img_if',
         'pub_key': 'obstacles_map_img_pub'},
    ]

    DATA_PRODUCTS = [OBSTACLES_IMG_DATA_PRODUCT,
                     GROUND_MAP_IMG_DATA_PRODUCT,
                     OBSTACLES_MAP_IMG_DATA_PRODUCT]

    # Never subscribe to our own overlay outputs as input image sources; skip
    # these product basenames even if the parent process's selected_sources
    # lists them (they resolve as real topics under the image namespace).
    OUTPUT_IMG_PRODUCTS = [OBSTACLES_IMG_DATA_PRODUCT,
                           GROUND_MAP_IMG_DATA_PRODUCT,
                           OBSTACLES_MAP_IMG_DATA_PRODUCT]

    node_if = None
    save_data_if = None

    img_node_dict = dict()
    img_node_lock = threading.Lock()

    sources_info_dict = dict()
    imgs_info_lock = threading.Lock()

    # Recent source frames per topic, newest last, as (stamp, raw Image msg).
    # Held outside sources_info_dict on purpose: that dict is deep-copied on a
    # 1 Hz timer, and copying full frames on every tick would cost more than the
    # alignment saves.
    img_buffer_dict = dict()
    img_buffer_lock = threading.Lock()

    state_str_msg = 'Loading'

    clear_det_time = 1.0

    last_status_time = None

    data_products = DATA_PRODUCTS

    min_range_m = 0.0
    max_range_m = 100.0

    has_color_image = False
    show_sources_enabled = True
    show_ground_enabled = False
    show_obstacles_enabled = False

    ground_transparency = DEFAULT_GROUND_TRANSPARENCY
    obstacles_transparency = DEFAULT_OBSTACLES_TRANSPARENCY

    overlay_labels = True
    overlay_range_bearing = True

    DEFAULT_NODE_NAME = "obstacles_img_pub"  # Can be overwritten by launch command

    connected = False

    watchdog_timeout = None

    def __init__(self):
        ####  NODE INIT SETUP ####
        nepi_sdk.init_node(name = self.DEFAULT_NODE_NAME)
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################
        # Create Msg Class
        self.msg_if = MsgIF(log_name = self.class_name)
        self.msg_if.pub_info("Starting Node Initialization Processes")

        ##############################
        # Init Class Variables

        # This node is launched by the parent obstacles node as
        # <parent node namespace>_img_pub, so stripping the suffix recovers the
        # parent's namespace.
        self.process_namespace = self.node_namespace.replace("_img_pub", "")
        self.obstacles_namespace = nepi_sdk.create_namespace(self.process_namespace, 'obstacles')
        self.msg_if.pub_info("Starting with Process Namespace: " + str(self.process_namespace))

        self.status_msg = ProcessStatus()
        self.enabled = False
        self.state_str_msg = "Unknown"
        self.max_image_pub_rate_hz = 10
        self.use_last_image = True

        self.imaging_enabled = True

        self.selected_source_topics = []

        ##############################
        # Create NodeClassIF Class

        # Configs Dict ########################
        # This node holds no configuration of its own -- every setting it
        # honors arrives on the parent's status message.
        self.CONFIGS_DICT = None

        # Params Config Dict ####################
        self.PARAMS_DICT = None

        # Services Config Dict ####################
        self.SRVS_DICT = None

        # Pubs Config Dict ####################
        self.PUBS_DICT = None

        # Subs Config Dict ####################
        self.SUBS_DICT = {
            'obstacles_status_sub': {
                'msg': ObstaclesStatus,
                'namespace': self.obstacles_namespace,
                'topic': 'status',
                'qsize': 10,
                'callback': self.statusCb,
                'callback_args': ()
            },
            'obstacles_data_sub': {
                'msg': Obstacles,
                'namespace': self.process_namespace,
                'topic': 'obstacles',
                'qsize': 10,
                'callback': self.obstaclesCb,
                'callback_args': ()
            },
            # The segmentation maps arrive on their own topic so that consumers
            # of the obstacle list do not carry the images. This node wants
            # both, so it subscribes to both. The parent publishes the pair back
            # to back from one process cycle, so the maps land within one
            # message of the obstacle list they were derived from.
            'obstacles_depth_map_sub': {
                'msg': ObstaclesDepthMap,
                'namespace': self.process_namespace,
                'topic': 'obstacles_depth_map',
                'qsize': 1,
                'callback': self.obstaclesDepthMapCb,
                'callback_args': ()
            },
        }

        # Create Node Class ####################
        self.node_if = NodeClassIF(
                        configs_dict = self.CONFIGS_DICT,
                        params_dict = self.PARAMS_DICT,
                        services_dict = self.SRVS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        msg_if = self.msg_if
                        )
        self.node_if.wait_for_ready()

        ###############################
        # Create System IFs

        # Setup Save Data IF. pub_status is False because the parent node owns
        # the save_data status topic on this same namespace; this instance is
        # only here so the image data product honors the same save commands.
        factory_data_rates = {}
        for d in self.data_products:
            factory_data_rates[d] = [1.0, 0.0, 100]

        self.save_data_if = SaveDataIF(namespace = self.process_namespace,
                        data_products = self.data_products,
                        pub_status = False,
                        factory_rate_dict = factory_data_rates,
                        msg_if = self.msg_if,
                        node_if = self.node_if
                        )
        nepi_sdk.sleep(1)

        ##########################
        # Complete Initialization

        # Start Timer Processes
        nepi_sdk.start_timer_process((1.0), self.updaterCb, oneshot = True)
        self.last_status_time = nepi_utils.get_time()
        nepi_sdk.start_timer_process(1, self.watchdogCb, oneshot = True)
        nepi_sdk.on_shutdown(self.shutdownCb)

        #########################################################
        ## Initiation Complete
        self.msg_if.pub_info("Initialization Complete")
        # Spin forever
        nepi_sdk.spin()
        #########################################################


    ###############################
    # Class Private Methods
    ###############################

    def getImgInfoDict(self):
        self.imgs_info_lock.acquire()
        sources_info_dict = copy.deepcopy(self.sources_info_dict)
        self.imgs_info_lock.release()
        return sources_info_dict

    def getActiveImgTopics(self):
        sources_info_dict = self.getImgInfoDict()
        active_source_topics = []
        for source_topic in sources_info_dict.keys():
            if sources_info_dict[source_topic].get('active', False) == True:
                active_source_topics.append(source_topic)
        return active_source_topics

    def mapSourceTopic(self, source_topic):
        # The parent process selects depth map topics; this node renders on the
        # sibling colour image when one exists. statusCb applies the same
        # substitution to selected_sources, so obstacle messages -- which carry
        # the parent's depth map topic -- have to be mapped the same way before
        # they can be looked up in sources_info_dict.
        if self.has_color_image == True:
            return source_topic.replace('depth_map', 'color_image')
        return source_topic

    def createImgInfoDict(self, pub_namespace):
        img_info_dict = dict()
        img_info_dict['active'] = True
        img_info_dict['img_connected'] = False
        img_info_dict['img_published'] = False
        img_info_dict['status_dict'] = None
        img_info_dict['pub_namespace'] = pub_namespace

        img_info_dict['connected'] = False
        img_info_dict['publishing'] = False
        img_info_dict['get_latency_time'] = 0
        img_info_dict['pub_latency_time'] = 0
        img_info_dict['process_time'] = 0
        img_info_dict['last_img_time'] = 0
        img_info_dict['last_det_time'] = 0

        # Written by obstaclesCb, read by imageCb. Every key imageCb reads has
        # to exist from the moment the topic is subscribed, because obstacle
        # messages and image messages arrive independently. img_stamp is the
        # source frame stamp the current obstacle data was derived from -- what
        # imageCb aligns the published image against.
        img_info_dict['obstacles_dict_list'] = []
        img_info_dict['navpose_dict'] = None
        img_info_dict['depth_map_ground'] = None
        img_info_dict['depth_map_obstacles'] = None
        img_info_dict['img_stamp'] = 0

        # One "first publish logged" flag per segmentation product, keyed by
        # data product name, so each new topic announces itself once the way
        # obstacles_image does through img_published.
        img_info_dict['segment_img_published'] = dict()
        for product in self.SEGMENT_IMG_PRODUCTS:
            img_info_dict['segment_img_published'][product['data_product']] = False

        return img_info_dict

    ###############.########################
    # Source frame buffer

    def bufferImgMsg(self, source_topic, img_msg, timestamp):
        # Every arriving frame goes in, not just the ones a publish cycle lands
        # on, because the aligned lookup can only find the frame the obstacle
        # data came from if that frame was kept.
        self.img_buffer_lock.acquire()
        img_buffer = self.img_buffer_dict.get(source_topic, None)
        if img_buffer is None:
            img_buffer = []
            self.img_buffer_dict[source_topic] = img_buffer
        img_buffer.append((timestamp, img_msg))
        # Age first, against the newest stamp, then the hard count cap.
        while len(img_buffer) > 1 and (timestamp - img_buffer[0][0]) > MAX_IMG_BUFFER_SEC:
            img_buffer.pop(0)
        while len(img_buffer) > MAX_IMG_BUFFER_LEN:
            img_buffer.pop(0)
        self.img_buffer_lock.release()

    def getAlignedImgMsg(self, source_topic, target_stamp):
        # The buffered frame closest in time to the frame the obstacle data was
        # derived from. Nearest rather than exact match because a source's colour
        # image and depth map are separate products and their stamps are only
        # equal when the driver publishes them from one capture -- nearest gives
        # the exact frame when they agree and the best available frame when they
        # do not. Returns None when there is nothing buffered or no data stamp to
        # align to yet, and the caller falls back to the frame in hand.
        if target_stamp is None or float(target_stamp) <= 0:
            return None
        target_stamp = float(target_stamp)
        best_msg = None
        best_delta = None
        self.img_buffer_lock.acquire()
        img_buffer = self.img_buffer_dict.get(source_topic, [])
        for entry in img_buffer:
            delta = abs(entry[0] - target_stamp)
            if best_delta is None or delta < best_delta:
                best_delta = delta
                best_msg = entry[1]
        self.img_buffer_lock.release()
        return best_msg

    def clearImgBuffer(self, source_topic):
        self.img_buffer_lock.acquire()
        if source_topic in self.img_buffer_dict.keys():
            self.img_buffer_dict[source_topic] = []
        self.img_buffer_lock.release()

    def updaterCb(self, timer):
        selected_source_topics = copy.deepcopy(self.selected_source_topics)
        active_source_topics = self.getActiveImgTopics()

        purge_list = []
        if self.imaging_enabled == False:
            purge_list = copy.deepcopy(list(self.sources_info_dict.keys()))
        else:
            # Update Image subscribers
            found_source_topics = []
            for source_topic in selected_source_topics:
                if os.path.basename(source_topic) in self.OUTPUT_IMG_PRODUCTS:
                    continue
                source_topic = nepi_sdk.find_topic(source_topic, exact = True)
                if source_topic != '':
                    found_source_topics.append(source_topic)
                    if source_topic not in active_source_topics:
                        self.msg_if.pub_info('Will subscribe to image topic: ' + source_topic)
                        self.subscribeImgTopic(source_topic)

            # Update Image Subs purge list
            active_source_topics = self.getActiveImgTopics()
            for source_topic in active_source_topics:
                if source_topic not in found_source_topics:
                    purge_list.append(source_topic)

        # Do image sub purging if required
        active_source_topics = self.getActiveImgTopics()
        for source_topic in purge_list:
            if source_topic not in active_source_topics:
                continue
            self.msg_if.pub_info('Will unsubscribe from image topic: ' + source_topic)
            self.unsubscribeImgTopic(source_topic)

        nepi_sdk.start_timer_process((1), self.updaterCb, oneshot = True)

    def watchdogCb(self, timer):
        cur_time = nepi_utils.get_time()
        elapsed = cur_time - self.last_status_time
        if self.watchdog_timeout is None:
            self.watchdog_timeout = WATCHDOG_TIMEOUT
            nepi_sdk.sleep(WATCHDOG_DELAY)
        else:
            if elapsed > WATCHDOG_TIMEOUT:
                msg = "Lost connection to parent node status msg.  Shutting down"
                self.msg_if.pub_warn(msg)
                nepi_sdk.signal_shutdown(msg)
                return

        nepi_sdk.start_timer_process(1, self.watchdogCb, oneshot = True)

    def subscribeImgTopic(self, source_topic):
        if source_topic == "None" or source_topic == "":
            return False

        img_source_namespace = os.path.dirname(source_topic)
        pub_namespace = img_source_namespace
        img_pub_topic = os.path.join(pub_namespace, self.OBSTACLES_IMG_DATA_PRODUCT)

        sources_info_dict = self.getImgInfoDict()
        if source_topic in sources_info_dict.keys():
            if sources_info_dict[source_topic]['active'] == True:
                return False
            self.img_node_lock.acquire()
            if source_topic in self.img_node_dict.keys():
                self.img_node_dict[source_topic]['img_pub'] = nepi_sdk.create_publisher(img_pub_topic, Image, queue_size = 1, log_name_list = [])
                for product in self.SEGMENT_IMG_PRODUCTS:
                    segment_pub_topic = os.path.join(pub_namespace, product['data_product'])
                    self.img_node_dict[source_topic][product['pub_key']] = nepi_sdk.create_publisher(segment_pub_topic, Image, queue_size = 1, log_name_list = [])
                nepi_sdk.sleep(1)
                self.img_node_dict[source_topic]['img_sub'] = nepi_sdk.create_subscriber(source_topic, Image, self.imageCb, queue_size = 1, callback_args = (source_topic), log_name_list = [])
                img_status_topic = nepi_sdk.create_namespace(source_topic, 'status')
                self.img_node_dict[source_topic]['img_status_sub'] = nepi_sdk.create_subscriber(img_status_topic, ImageStatus, self.imageStatusCb, queue_size = 1, callback_args = (source_topic), log_name_list = [])
                self.img_node_dict[source_topic]['img_if'].register_pubs()
                for product in self.SEGMENT_IMG_PRODUCTS:
                    segment_if = self.img_node_dict[source_topic][product['if_key']]
                    if segment_if is not None:
                        segment_if.register_pubs()
            self.img_node_lock.release()
            self.imgs_info_lock.acquire()
            self.sources_info_dict[source_topic]['active'] = True
            self.imgs_info_lock.release()
            return True

        self.msg_if.pub_info('Publishing image ' + source_topic + ' on namespace: ' + img_pub_topic)

        img_pub = nepi_sdk.create_publisher(img_pub_topic, Image, queue_size = 1, log_name_list = [])
        nepi_sdk.sleep(1)
        img_sub = nepi_sdk.create_subscriber(source_topic, Image, self.imageCb, queue_size = 1, callback_args = (source_topic), log_name_list = [])
        img_status_topic = nepi_sdk.create_namespace(source_topic, 'status')
        img_status_sub = nepi_sdk.create_subscriber(img_status_topic, ImageStatus, self.imageStatusCb, queue_size = 1, callback_args = (source_topic), log_name_list = [])

        # Create obstacles image publisher
        img_if = ColorImageIF(namespace = pub_namespace,
                        data_product = self.OBSTACLES_IMG_DATA_PRODUCT,
                        data_source_description = 'image',
                        data_ref_description = 'image',
                        perspective = 'pov',
                        save_data_if = self.save_data_if,
                        init_overlay_text_list = [],
                        live_adjustments_disabled = True,
                        aspect_adjustment_disabled = True,
                        log_name = self.OBSTACLES_IMG_DATA_PRODUCT,
                        log_name_list = [],
                        msg_if = self.msg_if)

        # Create the two segmentation image publishers beside the overlay one.
        # Same ColorImageIF pattern, same save_data_if, same pub_namespace --
        # only the data product name differs, so all three appear and disappear
        # together with their source.
        segment_nodes_dict = dict()
        for product in self.SEGMENT_IMG_PRODUCTS:
            data_product = product['data_product']
            segment_pub_topic = os.path.join(pub_namespace, data_product)
            self.msg_if.pub_info('Publishing image ' + source_topic + ' on namespace: ' + segment_pub_topic)
            segment_nodes_dict[product['pub_key']] = nepi_sdk.create_publisher(segment_pub_topic, Image, queue_size = 1, log_name_list = [])
            segment_nodes_dict[product['if_key']] = ColorImageIF(namespace = pub_namespace,
                            data_product = data_product,
                            data_source_description = 'image',
                            data_ref_description = 'image',
                            perspective = 'pov',
                            save_data_if = self.save_data_if,
                            init_overlay_text_list = [],
                            live_adjustments_disabled = True,
                            aspect_adjustment_disabled = True,
                            log_name = data_product,
                            log_name_list = [],
                            msg_if = self.msg_if)

        self.img_node_lock.acquire()
        self.img_node_dict[source_topic] = {
                                        'img_sub': img_sub,
                                        'img_status_sub': img_status_sub,
                                        'img_pub': img_pub,
                                        'img_if': img_if,
                                        }
        self.img_node_dict[source_topic].update(segment_nodes_dict)
        self.img_node_lock.release()

        self.imgs_info_lock.acquire()
        self.sources_info_dict[source_topic] = self.createImgInfoDict(pub_namespace)
        self.imgs_info_lock.release()

        return True

    def unsubscribeImgTopic(self, source_topic):
        if source_topic not in self.sources_info_dict.keys():
            return False
        if self.sources_info_dict[source_topic]['active'] == False:
            return False

        self.msg_if.pub_info('Unsubscribing from image topic: ' + source_topic)

        self.img_node_lock.acquire()
        if source_topic in self.img_node_dict.keys():
            if self.img_node_dict[source_topic]['img_sub'] is not None:
                self.img_node_dict[source_topic]['img_sub'].unregister()
            if self.img_node_dict[source_topic]['img_status_sub'] is not None:
                self.img_node_dict[source_topic]['img_status_sub'].unregister()
            if self.img_node_dict[source_topic]['img_pub'] is not None:
                self.img_node_dict[source_topic]['img_pub'].unregister()
            if self.img_node_dict[source_topic]['img_if'] is not None:
                self.img_node_dict[source_topic]['img_if'].unregister_pubs()
            for product in self.SEGMENT_IMG_PRODUCTS:
                segment_pub = self.img_node_dict[source_topic].get(product['pub_key'], None)
                if segment_pub is not None:
                    segment_pub.unregister()
                segment_if = self.img_node_dict[source_topic].get(product['if_key'], None)
                if segment_if is not None:
                    segment_if.unregister_pubs()
            nepi_sdk.sleep(1)
            self.img_node_dict[source_topic]['img_sub'] = None
            self.img_node_dict[source_topic]['img_status_sub'] = None
            self.img_node_dict[source_topic]['img_pub'] = None
            for product in self.SEGMENT_IMG_PRODUCTS:
                self.img_node_dict[source_topic][product['pub_key']] = None
        self.img_node_lock.release()

        self.imgs_info_lock.acquire()
        self.sources_info_dict[source_topic]['active'] = False
        self.sources_info_dict[source_topic]['status_dict'] = None
        self.sources_info_dict[source_topic]['connected'] = False
        self.sources_info_dict[source_topic]['publishing'] = False
        self.sources_info_dict[source_topic]['img_connected'] = False
        self.sources_info_dict[source_topic]['img_published'] = False
        self.imgs_info_lock.release()

        # Nothing in the buffer survives an unsubscribe -- on resubscribe those
        # frames would be arbitrarily old and could win an alignment lookup.
        self.clearImgBuffer(source_topic)

        return True

    def needsImgCheck(self, source_topic, if_key = None):
        # if_key None asks about every published product for this source; a key
        # asks about that one. The IF's needs_data is a level flag its own timer
        # refreshes from subscriber and save state, so polling it is free.
        if_keys = [if_key]
        if if_key is None:
            if_keys = ['img_if'] + [p['if_key'] for p in self.SEGMENT_IMG_PRODUCTS]
        needs_img = False
        self.img_node_lock.acquire()
        if source_topic in self.img_node_dict.keys():
            for key in if_keys:
                img_if = self.img_node_dict[source_topic].get(key, None)
                if img_if is not None and img_if.needs_data_check() == True:
                    needs_img = True
                    break
        self.img_node_lock.release()
        return needs_img

    def imageStatusCb(self, status_msg, args):
        source_topic = args
        if source_topic not in self.sources_info_dict.keys():
            return
        self.imgs_info_lock.acquire()
        if source_topic in self.sources_info_dict.keys():
            status_dict = nepi_sdk.convert_msg2dict(status_msg)
            if self.sources_info_dict[source_topic]['status_dict'] is None:
                self.msg_if.pub_info('Connected to image status topic: ' + source_topic + '/status')
            self.sources_info_dict[source_topic]['status_dict'] = status_dict
        self.imgs_info_lock.release()

    def imageCb(self, image_msg, args):
        source_topic = args

        if source_topic not in self.sources_info_dict.keys():
            return

        if self.sources_info_dict[source_topic]['img_connected'] == False:
            self.msg_if.pub_info('Connected to image topic: ' + source_topic)
        self.sources_info_dict[source_topic]['img_connected'] = True

        # Any of the three published products wanting data is enough to run the
        # cycle; publishImgData then skips the individual products nobody is
        # watching, so an unwatched product still costs nothing.
        needs_img = self.needsImgCheck(source_topic)

        if needs_img == False or self.imaging_enabled == False:
            return

        sel_imgs = copy.deepcopy(self.selected_source_topics)
        max_image_pub_rate_hz = copy.deepcopy(self.max_image_pub_rate_hz)
        if source_topic not in sel_imgs or max_image_pub_rate_hz <= 0.01:
            return

        if self.sources_info_dict[source_topic]['connected'] == False:
            self.msg_if.pub_info("Got image topic: " + str(source_topic))
        self.sources_info_dict[source_topic]['connected'] = True

        if self.enabled == False or self.state_str_msg != ProcessStatus.STATE_PROCESSING:
            return

        timestamp = copy.deepcopy(float(image_msg.header.stamp.to_sec()))
        self.bufferImgMsg(source_topic, image_msg, timestamp)

        # Check if time to publish
        delay_time = float(1) / max_image_pub_rate_hz
        last_img_time = self.sources_info_dict[source_topic]['last_img_time']
        current_time = nepi_utils.get_time()
        if round((current_time - last_img_time), 3) <= delay_time:
            return
        self.sources_info_dict[source_topic]['last_img_time'] = current_time

        self.sources_info_dict[source_topic]['get_latency_time'] = (nepi_utils.get_time() - timestamp)

        obstacles_dict_list = copy.deepcopy(self.sources_info_dict[source_topic]['obstacles_dict_list'])
        if obstacles_dict_list is None:
            obstacles_dict_list = []

        depth_map_ground = copy.deepcopy(self.sources_info_dict[source_topic]['depth_map_ground'])
        depth_map_obstacles = copy.deepcopy(self.sources_info_dict[source_topic]['depth_map_obstacles'])
        obstacles_stamp = self.sources_info_dict[source_topic]['img_stamp']

        # use_last_image is the "align the image with the obstacle data" control.
        # It used to publish whatever frame the previous publish cycle stashed,
        # which is one whole source frame in arrears no matter how far behind the
        # data actually is -- so on a slow source the boxes ran seconds ahead of
        # the image they were drawn on. Now it looks up the frame the data was
        # derived from by stamp, which is correct at any source rate. Off means no
        # alignment at all: newest frame, data as it stands.
        use_img_msg = image_msg
        if self.use_last_image == True:
            aligned_img_msg = self.getAlignedImgMsg(source_topic, obstacles_stamp)
            if aligned_img_msg is not None:
                use_img_msg = aligned_img_msg

        # Report the stamp of the frame actually published, not of the frame that
        # triggered the cycle -- otherwise an aligned publish claims to be fresher
        # than it is and every downstream latency reads low.
        use_timestamp = copy.deepcopy(float(use_img_msg.header.stamp.to_sec()))
        use_cv2_img = nepi_img.rosimg_to_cv2img(use_img_msg)

        if use_cv2_img is not None:
            self.processObstaclesImage(source_topic,
                                        use_cv2_img,
                                        obstacles_dict_list,
                                        depth_map_ground,
                                        depth_map_obstacles,
                                        timestamp = use_timestamp,
                                        )
            self.sources_info_dict[source_topic]['pub_latency_time'] = (nepi_utils.get_time() - use_timestamp)

    def processObstaclesImage(self, source_topic,
                                    cv2_img,
                                    obstacles_dict_list,
                                    depth_map_ground,
                                    depth_map_obstacles,
                                    timestamp = None):
        if source_topic not in self.sources_info_dict.keys():
            return False

        self.sources_info_dict[source_topic]['publishing'] = True
        status_dict = copy.deepcopy(self.sources_info_dict[source_topic]['status_dict'])
        if status_dict is not None:
            width_pixel = status_dict['width_px']
            height_pixel = status_dict['height_px']
            width_deg = status_dict['width_deg']
            height_deg = status_dict['height_deg']
        else:
            width_pixel = 0
            height_pixel = 0
            width_deg = 100
            height_deg = 70

        if cv2_img is None and width_pixel > 0 and height_pixel > 0:
            cv2_img = nepi_img.create_blank_image((height_pixel, width_pixel, 3))
        if cv2_img is None:
            return False

        if depth_map_ground is not None and self.show_ground_enabled == True:
            cv2_img = self.applyDepthMapOverlay(depth_map_ground, cv2_img,
                                                self.getBlendRatio(self.ground_transparency),
                                                color = GROUND_OVERLAY_COLOR)

        if depth_map_obstacles is not None and self.show_obstacles_enabled == True:
            cv2_img = self.applyDepthMapOverlay(depth_map_obstacles, cv2_img,
                                                self.getBlendRatio(self.obstacles_transparency),
                                                color = OBSTACLES_OVERLAY_COLOR)

        if len(obstacles_dict_list) > 0 and self.show_obstacles_enabled == True:
            cv2_img = self.applyBoxesOverlay(obstacles_dict_list, cv2_img, OBSTACLE_BOX_COLOR)

        self.publishImgData(source_topic,
                            cv2_img,
                            width_deg = width_deg,
                            height_deg = height_deg,
                            timestamp = timestamp,
                            add_overlay_text_list = []
                            )

        if self.sources_info_dict[source_topic]['img_published'] == False:
            namespace = self.sources_info_dict[source_topic]['pub_namespace']
            self.msg_if.pub_info('Published image topic: ' + os.path.join(namespace, self.OBSTACLES_IMG_DATA_PRODUCT))
        self.sources_info_dict[source_topic]['img_published'] = True

        # The two segmentation renders ride the same cycle as the overlay, so
        # they obey set_image_pub, max_image_pub_rate_hz, use_last_image and the
        # process state gate without a second rate path of their own.
        self.publishSegmentImgs(source_topic,
                                depth_map_ground,
                                depth_map_obstacles,
                                width_deg = width_deg,
                                height_deg = height_deg,
                                timestamp = timestamp)

        return True

    def publishSegmentImgs(self, source_topic,
                                 depth_map_ground,
                                 depth_map_obstacles,
                                 width_deg = 100,
                                 height_deg = 70,
                                 timestamp = None):
        if source_topic not in self.sources_info_dict.keys():
            return False

        depth_maps = {
            'depth_map_ground': depth_map_ground,
            'depth_map_obstacles': depth_map_obstacles,
        }
        namespace = self.sources_info_dict[source_topic]['pub_namespace']

        for product in self.SEGMENT_IMG_PRODUCTS:
            data_product = product['data_product']
            if self.needsImgCheck(source_topic, if_key = product['if_key']) == False:
                continue
            # A map the process did not produce this cycle arrives as None -- the
            # parent sends a 0x0 Image and getDepthMapFromMsg turns that back into
            # None. Skip the publish rather than putting an empty image on the wire.
            cv2_img = self.getSegmentColorImg(depth_maps.get(product['map_key'], None))
            if cv2_img is None:
                continue
            self.publishImgData(source_topic,
                                cv2_img,
                                width_deg = width_deg,
                                height_deg = height_deg,
                                timestamp = timestamp,
                                add_overlay_text_list = [],
                                img_if_key = product['if_key'],
                                img_pub_key = product['pub_key'])
            if self.sources_info_dict[source_topic]['segment_img_published'].get(data_product, False) == False:
                self.msg_if.pub_info('Published image topic: ' + os.path.join(namespace, data_product))
                self.sources_info_dict[source_topic]['segment_img_published'][data_product] = True

        return True

    def getSegmentColorImg(self, np_depth_map):
        # Colourize one half of the per-cycle segmentation for viewing, using the
        # platform depth map colorizer rather than a routine of this app's own.
        # Non-member pixels are NaN by contract, so isfinite IS the segment mask;
        # they are painted SEGMENT_NONE_COLOR afterwards because the colorizer
        # folds NaN onto the max-range colour, which a viewer cannot tell from a
        # real far return. The colorizer rewrites its input in place, hence the
        # deep copy -- the caller's map is still needed by the overlay path.
        if np_depth_map is None:
            return None
        try:
            mask = np.isfinite(np_depth_map)
            if mask.any() == False:
                return None
            cv2_img = nepi_img.npDepthMap_to_cv2ColorImg(copy.deepcopy(np_depth_map),
                                                        min_range_m = self.min_range_m,
                                                        max_range_m = self.max_range_m)
            if cv2_img is None:
                return None
            cv2_img[np.logical_not(mask)] = SEGMENT_NONE_COLOR
            return cv2_img
        except Exception as e:
            self.msg_if.pub_warn("Failed to colorize segmentation depth map: " + str(e), throttle_s = 5.0)
            return None

    def publishImgData(self, source_topic, cv2_img, encoding = "bgr8", timestamp = None,
                        width_deg = 100,
                        height_deg = 70,
                        add_overlay_text_list = [],
                        img_if_key = 'img_if',
                        img_pub_key = 'img_pub'):
        if self.imaging_enabled == False:
            return
        # try/finally: any raise between acquire and release (a
        # publish_cv2_img signature drift, a missing dict key) would
        # otherwise leave the lock held and wedge image publishing for
        # the life of the node. Degrade to a logged error instead.
        self.img_node_lock.acquire()
        try:
            img_if = self.img_node_dict[source_topic][img_if_key]
            img_pub = self.img_node_dict[source_topic][img_pub_key]

            # Per-product subscriber gate. imageCb runs the cycle when ANY of the
            # three products wants data, so an unwatched product still has to opt
            # out here or it would pay for a neighbour's subscriber.
            if img_if is not None and img_if.ready == True and img_if.needs_data_check() == False:
                return

            if img_if is None or img_if.ready == False:
                img_msg = nepi_img.cv2img_to_rosimg(cv2_img, encoding = encoding)
                nepi_sdk.publish_pub(img_pub, img_msg)
            else:
                img_if.publish_cv2_img(cv2_img,
                                    encoding = encoding,
                                    timestamp = timestamp,
                                    width_deg = width_deg,
                                    height_deg = height_deg,
                                    min_range_m = self.min_range_m,
                                    max_range_m = self.max_range_m,
                                    add_overlay_text_list = add_overlay_text_list
                                    )
        except Exception as e:
            self.msg_if.pub_warn("Failed to publish image for source: " + str(source_topic) + " : " + str(e), throttle_s = 5.0)
        finally:
            self.img_node_lock.release()

    def getBlendRatio(self, transparency):
        # The wire value is transparency, the way the RUI slider labels it;
        # addWeighted wants the overlay's share, which is its complement.
        try:
            transparency = float(transparency)
        except (TypeError, ValueError):
            return 0.5
        if transparency < 0.0:
            transparency = 0.0
        elif transparency > 1.0:
            transparency = 1.0
        return 1.0 - transparency

    def applyDepthMapOverlay(self, np_depth_map, cv2_img, blend_ratio, color = None):
        # Blend the segmentation only where the raster has a finite range value.
        # Non-member pixels are NaN by contract, so the mask is exactly the
        # segment. With a color the segment is painted that one flat colour; with
        # color None it is colourized by range.
        try:
            mask = np.isfinite(np_depth_map)
            if mask.any() == False:
                return cv2_img
            if color is not None:
                cv2_map = np.zeros((np_depth_map.shape[0], np_depth_map.shape[1], 3), dtype = np.uint8)
                cv2_map[:, :] = color
            else:
                cv2_map = nepi_img.npDepthMap_to_cv2ColorImg(copy.deepcopy(np_depth_map),
                                                            min_range_m = self.min_range_m,
                                                            max_range_m = self.max_range_m)
            if cv2_map is None:
                return cv2_img
            if cv2_map.shape[:2] != cv2_img.shape[:2]:
                cv2_map = cv2.resize(cv2_map, (cv2_img.shape[1], cv2_img.shape[0]))
                mask = cv2.resize(mask.astype(np.uint8), (cv2_img.shape[1], cv2_img.shape[0])) > 0
            blended = cv2.addWeighted(cv2_img, (1.0 - blend_ratio), cv2_map, blend_ratio, 0)
            cv2_out = copy.deepcopy(cv2_img)
            cv2_out[mask] = blended[mask]
            return cv2_out
        except Exception as e:
            self.msg_if.pub_warn("Failed to apply depth map overlay: " + str(e), throttle_s = 5.0)
            return cv2_img

    def applyBoxesOverlay(self, boxes_dict_list, cv2_img, default_color):
        cv2_obstacles_img = copy.deepcopy(cv2_img)
        cv2_shape = cv2_img.shape
        img_width = cv2_shape[1]
        img_height = cv2_shape[0]
        img_size = cv2_shape[:2]

        font = cv2.FONT_HERSHEY_DUPLEX
        scale = 1.5e-3 - 0.1e-3 * math.ceil(max([img_height, img_width]) / 700)
        fontScale, fontThickness = nepi_img.optimal_font_dims(cv2_img, font_scale = scale, thickness_scale = scale)
        fontColor = (255, 255, 255)
        lineType = cv2.LINE_AA
        line_thickness = 1 + math.ceil(max([img_height, img_width]) / 2000)

        for box_dict in boxes_dict_list:
            ###### Apply Image Overlays and Publish Image ROS Message
            class_name = box_dict['name']
            xmin = box_dict['xmin']
            ymin = box_dict['ymin']
            xmax = box_dict['xmax']
            ymax = box_dict['ymax']

            if xmin <= 0:
                xmin = 5
            if ymin <= 0:
                ymin = 5
            if xmax >= img_size[1]:
                xmax = img_size[1] - 5
            if ymax >= img_size[0]:
                ymax = img_size[0] - 5

            bot_left_box = (xmin, ymin)
            top_right_box = (xmax, ymax)

            class_color = default_color

            success = False
            try:
                cv2.rectangle(cv2_obstacles_img, bot_left_box, top_right_box, class_color, thickness = line_thickness)
                success = True
            except Exception as e:
                self.msg_if.pub_warn("Failed to create bounding box rectangle: " + str(e), throttle_s = 5.0)

            if success == False:
                continue

            ## Overlay Text
            overlay_text = ""
            if self.overlay_labels:
                overlay_text = overlay_text + str(class_name) + " "
            if self.overlay_range_bearing:
                rb_text = ''
                if box_dict['range_m'] != -999 and box_dict['range_m'] != '':
                    rb_text = rb_text + str(round(box_dict['range_m'], 1)) + 'm :'
                if box_dict['azimuth_deg'] != -999 and box_dict['elevation_deg'] != -999:
                    rb_text = rb_text + str(round(box_dict['azimuth_deg'], 1)) + 'deg '
                    rb_text = rb_text + str(round(box_dict['elevation_deg'], 1)) + 'deg '
                if len(rb_text) > 0:
                    overlay_text = overlay_text + rb_text

            if len(overlay_text) == 0:
                continue

            text_size = cv2.getTextSize(overlay_text, font, fontScale, fontThickness)
            line_height = text_size[0][1]
            line_width = text_size[0][0]
            x_padding = int(line_height * 0.4)
            y_padding = int(line_height * 0.4)

            center = bot_left_box[0] + int((top_right_box[0] - bot_left_box[0]) / 2)
            bot_left_text = (center + x_padding, ymin - (line_thickness * 2) - y_padding)
            text_bot_left_box = (center - x_padding, bot_left_text[1] + y_padding)
            text_top_right_box = (center + line_width + x_padding, bot_left_text[1] - line_height - y_padding)
            box_color = (0, 0, 0)

            try:
                cv2.rectangle(cv2_obstacles_img, text_bot_left_box, text_top_right_box, box_color, -1)
                cv2.putText(cv2_obstacles_img, overlay_text,
                    bot_left_text,
                    font,
                    fontScale,
                    fontColor,
                    fontThickness,
                    lineType)
            except Exception as e:
                self.msg_if.pub_warn("Failed to apply overlay label text: " + str(e), throttle_s = 5.0)

        return cv2_obstacles_img

    def getBoxDict(self, entry_dict):
        return {
            'name': entry_dict.get('name', ''),
            'xmin': entry_dict.get('xmin_pixel', 0),
            'ymin': entry_dict.get('ymin_pixel', 0),
            'xmax': entry_dict.get('xmax_pixel', 0),
            'ymax': entry_dict.get('ymax_pixel', 0),
            'range_m': entry_dict.get('range_m', -999),
            'azimuth_deg': entry_dict.get('azimuth_deg', -999),
            'elevation_deg': entry_dict.get('elevation_deg', -999),
        }

    def getDepthMapFromMsg(self, img_msg):
        # A 0x0 Image is the parent's wire representation of "not produced
        # this cycle".
        if img_msg is None or img_msg.width == 0 or img_msg.height == 0:
            return None
        try:
            return nepi_img.rosimg_to_cv2img(img_msg)
        except Exception as e:
            self.msg_if.pub_warn("Failed to convert depth map msg: " + str(e), throttle_s = 5.0)
            return None

    def obstaclesCb(self, msg):
        self.connected = True
        source_topic = self.mapSourceTopic(msg.source_topic)
        if source_topic not in self.sources_info_dict.keys():
            return

        current_time = nepi_utils.get_time()
        # msg.obstacles is an Obstacle[] array -- convert_msg2dict takes a single
        # message, so convert per entry.
        obstacles_list = []
        for obstacle_msg in msg.obstacles:
            obstacles_list.append(nepi_sdk.convert_msg2dict(obstacle_msg))
        navpose_dict = nepi_sdk.convert_msg2dict(msg.navpose_msg)

        overlay_obstacles_list = []
        for obstacle in obstacles_list:
            overlay_obstacles_list.append(self.getBoxDict(obstacle))

        self.imgs_info_lock.acquire()
        if source_topic in self.sources_info_dict.keys():
            self.sources_info_dict[source_topic]['obstacles_dict_list'] = overlay_obstacles_list
            self.sources_info_dict[source_topic]['navpose_dict'] = navpose_dict
            self.sources_info_dict[source_topic]['img_stamp'] = msg.source_timestamp
            self.sources_info_dict[source_topic]['last_det_time'] = current_time
        self.imgs_info_lock.release()

    def obstaclesDepthMapCb(self, msg):
        # The segmentation half of one process cycle. Kept separate from
        # obstaclesCb because the two now arrive as separate messages; both write
        # under imgs_info_lock, so imageCb never reads a half-updated entry.
        source_topic = self.mapSourceTopic(msg.source_topic)
        if source_topic not in self.sources_info_dict.keys():
            return

        depth_map_ground = self.getDepthMapFromMsg(msg.depth_map_ground)
        depth_map_obstacles = self.getDepthMapFromMsg(msg.depth_map_obstacles)

        self.imgs_info_lock.acquire()
        if source_topic in self.sources_info_dict.keys():
            self.sources_info_dict[source_topic]['depth_map_ground'] = depth_map_ground
            self.sources_info_dict[source_topic]['depth_map_obstacles'] = depth_map_obstacles
        self.imgs_info_lock.release()

    def statusCb(self, msg):
        self.last_status_time = nepi_utils.get_time()

        self.status_msg = msg.process_status

        self.enabled = self.status_msg.enabled
        self.state_str_msg = self.status_msg.msg_str
        self.max_image_pub_rate_hz = self.status_msg.max_image_pub_rate_hz
        self.use_last_image = self.status_msg.use_last_image
        self.imaging_enabled = self.status_msg.image_pub_enabled

        self.min_range_m = msg.min_range_m
        self.max_range_m = msg.max_range_m

        self.has_color_image = msg.has_color_image
        self.show_sources_enabled = msg.show_sources_enabled
        self.show_ground_enabled = msg.show_ground_enabled
        self.show_obstacles_enabled = msg.show_obstacles_enabled

        self.ground_transparency = msg.ground_transparency
        self.obstacles_transparency = msg.obstacles_transparency

        last_sel_imgs = copy.deepcopy(self.selected_source_topics)
        selected_source_topics = []
        for topic in self.status_msg.selected_sources:
            selected_source_topics.append(self.mapSourceTopic(topic))

        self.selected_source_topics = selected_source_topics
        if last_sel_imgs != self.selected_source_topics:
            self.msg_if.pub_info("Updating selected images topics: " + str(self.selected_source_topics))

    def shutdownCb(self):
        for source_topic in list(self.sources_info_dict.keys()):
            try:
                self.unsubscribeImgTopic(source_topic)
            except Exception:
                pass


#########################################
# Main
#########################################
if __name__ == '__main__':
    ObstaclesImgPub()
