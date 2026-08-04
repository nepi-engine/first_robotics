#!/usr/bin/env python
##
## Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
##
## This file is part of nepi-engine
## (see https://github.com/nepi-engine).
##
## License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
##

import time
import copy

from std_msgs.msg import Bool, Empty, String, Float32

from nepi_app_obstacles.msg import NepiAppObstaclesStatus

from nepi_sdk import nepi_sdk

from nepi_api.node_if import NodeClassIF
from nepi_api.messages_if import MsgIF

from nepi_api.connect_data_if import ConnectDepthMapIF
from nepi_api.connect_data_if import ConnectColorImageIF
from nepi_api.connect_data_if import ConnectNavPoseIF


#########################################
# Factory Control Values
FACTORY_ENABLED = False
FACTORY_SELECTED_OPTION = "None"
FACTORY_VALUE = 0.0
FACTORY_OPTIONS = ["None", "Option_A", "Option_B", "Option_C"]

STATUS_PUBLISH_RATE_HZ = 1.0

# Connect namespace names, one per connect IF. Each name is the <app>/<name>
# connect namespace its IF owns and the matching Nepi_IF_Connect* component in
# NepiAppObstacles.js binds to, so all three are named here to keep that
# node-to-RUI pairing greppable from one place. Depth map and NavPose match their
# class defaults; ConnectColorImageIF is a bare ConnectBaseImageIF subclass whose
# inherited default is the generic 'data_connect' (connect_data_if.CONNECT_NAME),
# which says nothing about what it carries, so it is named explicitly.
DEPTH_MAP_CONNECT_NAME = "depth_map_connect"
COLOR_IMAGE_CONNECT_NAME = "color_image_connect"
NAVPOSE_CONNECT_NAME = "navpose_connect"

# Subtopic a depth map's own image is published on, one level under the depth map
# itself: DepthMapIF with pub_image=True creates a DepthMapImageIF on its own
# namespace, and that IF's data product is 'depth_map_image'. Same join
# nepi_app_stereo_cam makes with DEPTH_IMAGE_SUBTOPIC. Used by
# getDepthMapImageTopic() -- see the comment there for why the depth map's own
# DepthMapStatus.image_topic is not taken at face value.
DEPTH_MAP_IMAGE_SUBTOPIC = 'depth_map_image'

#########################################
# Node Class
#########################################

class NepiObstaclesApp(object):

    enabled = FACTORY_ENABLED
    selected_option = FACTORY_SELECTED_OPTION
    value = FACTORY_VALUE
    options = FACTORY_OPTIONS

    node_if = None

    depth_map_if = None
    color_image_if = None
    navpose_if = None

    # Per-IF first-connection flags. Each connect IF's callback logs once on its
    # first invocation, then sets its flag.
    got_first_depth_map = False
    got_first_color_image = False
    got_first_navpose = False

    # Latest data dict and status msg per connect IF. Each callback stores both
    # on every invocation, so the rest of the app reads the most recent values
    # from here rather than re-querying the IF.
    depth_map_dict = None
    depth_map_status = None
    color_image_dict = None
    color_image_status = None
    navpose_dict = None
    navpose_status = None

    # Source topics as last published, keyed by status message field name. Each
    # connect callback compares the live set against this so a selection change
    # reaches the RUI at the callback rate instead of waiting out a full status
    # timer period.
    last_source_topics = None

    DEFAULT_NODE_NAME = "app_obstacles"

    def __init__(self):
        #### APP NODE INIT SETUP ####
        nepi_sdk.init_node(name=self.DEFAULT_NODE_NAME)
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################
        # Create Msg Class
        self.msg_if = MsgIF(log_name=self.class_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")

        ##############################
        # Initialize Class Variables

        ##############################
        ### Setup Node

        # Configs Config Dict ####################
        self.CFGS_DICT = {
            'init_callback': self.initCb,
            'reset_callback': self.resetCb,
            'factory_reset_callback': self.factoryResetCb,
            'init_configs': True,
            'namespace': self.node_namespace
        }

        # Params Config Dict ####################
        self.PARAMS_DICT = {
            'enabled': {
                'namespace': self.node_namespace,
                'factory_val': self.enabled
            },
            'selected_option': {
                'namespace': self.node_namespace,
                'factory_val': self.selected_option
            },
            'value': {
                'namespace': self.node_namespace,
                'factory_val': self.value
            }
        }

        # Publishers Config Dict ####################
        self.PUBS_DICT = {
            'status_pub': {
                'namespace': self.node_namespace,
                'topic': 'status',
                'msg': NepiAppObstaclesStatus,
                'qsize': 1,
                'latch': True
            }
        }

        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            'set_enabled': {
                'namespace': self.node_namespace,
                'topic': 'set_enabled',
                'msg': Bool,
                'qsize': 10,
                'callback': self.setEnabledCb,
                'callback_args': ()
            },
            'set_option': {
                'namespace': self.node_namespace,
                'topic': 'set_option',
                'msg': String,
                'qsize': 10,
                'callback': self.setOptionCb,
                'callback_args': ()
            },
            'set_value': {
                'namespace': self.node_namespace,
                'topic': 'set_value',
                'msg': Float32,
                'qsize': 10,
                'callback': self.setValueCb,
                'callback_args': ()
            },
            'trigger_action': {
                'namespace': self.node_namespace,
                'topic': 'trigger_action',
                'msg': Empty,
                'qsize': 10,
                'callback': self.triggerActionCb,
                'callback_args': ()
            }
        }

        # Create Node Class ####################
        self.node_if = NodeClassIF(
            configs_dict=self.CFGS_DICT,
            params_dict=self.PARAMS_DICT,
            pubs_dict=self.PUBS_DICT,
            subs_dict=self.SUBS_DICT
        )

        self.node_if.wait_for_ready()

        ##############################
        # Surface the source selectors this app's RUI page renders. Each connect
        # IF owns the <app>/<connect_name> connect namespace the matching
        # Nepi_IF_Connect* component binds to, built with show_selector=True and
        # the other show_* flags False -- this page selects sources, it does not
        # drive them.
        self.setupInterfaceIFs()

        ##############################
        self.initCb(do_updates=True)

        time.sleep(1)
        nepi_sdk.start_timer_process(float(1) / STATUS_PUBLISH_RATE_HZ, self.statusPublishCb)

        time.sleep(1)
        self.msg_if.pub_info("Initialization Complete")

        nepi_sdk.on_shutdown(self.cleanup_actions)
        nepi_sdk.spin()


    ###################
    ## App Callbacks

    def setEnabledCb(self, msg):
        self.msg_if.pub_info(str(msg))
        self.enabled = msg.data
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('enabled', self.enabled)
            self.node_if.save_config()

    def setOptionCb(self, msg):
        self.msg_if.pub_info(str(msg))
        option = msg.data
        if option in self.options:
            self.selected_option = option
            self.publish_status()
            if self.node_if is not None:
                self.node_if.set_param('selected_option', self.selected_option)
                self.node_if.save_config()

    def setValueCb(self, msg):
        self.msg_if.pub_info(str(msg))
        self.value = msg.data
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('value', self.value)
            self.node_if.save_config()

    def triggerActionCb(self, msg):
        # The app's action is to apply its current state: the selected option at
        # the current value. Disabled means the trigger is a no-op rather than a
        # silent one -- an operator pressing it deserves to know why nothing
        # happened. Status is republished so the RUI reflects the trigger.
        if self.enabled is False:
            self.msg_if.pub_warn("Action ignored: app is disabled")
            return
        self.msg_if.pub_info("Action triggered: " + str(self.selected_option) +
                             " = " + str(self.value))
        self.publish_status()


    #######################
    ### Config Functions

    def initCb(self, do_updates=False):
        if self.node_if is not None:
            self.enabled = self.node_if.get_param('enabled')
            self.selected_option = self.node_if.get_param('selected_option')
            self.value = self.node_if.get_param('value')
        if do_updates:
            pass
        self.publish_status()

    def resetCb(self, do_updates=True):
        self.msg_if.pub_warn("Resetting")
        if do_updates:
            pass
        self.initCb(do_updates=do_updates)

    def factoryResetCb(self, do_updates=True):
        self.msg_if.pub_warn("Factory Resetting")
        if do_updates:
            pass
        self.initCb(do_updates=do_updates)


    ###################
    ## Interface IF Setup

    def setupInterfaceIFs(self):
        # Connect-side (consumer) IFs, in RUI display order: depth map, color
        # image, NavPose. Each auto-discovers the matching producer topics and
        # exposes a selector panel (show_selector=True) with the data and controls
        # panels hidden. The operator selects a depth map, not a depth map image:
        # the depth map is the data product this app will process, and its
        # DepthMapStatus names the depth map image the RUI viewer mounts on, so no
        # separate image selector is needed. Each connect_name is passed
        # explicitly to keep the RUI binding greppable from here.
        self.depth_map_if = ConnectDepthMapIF(
                        connect_name = DEPTH_MAP_CONNECT_NAME,
                        show_selector = True,
                        show_controls = False,
                        show_data = False,
                        dataCB = self.depthMapConnectCb,
                        msg_if = self.msg_if)

        self.color_image_if = ConnectColorImageIF(
                        connect_name = COLOR_IMAGE_CONNECT_NAME,
                        show_selector = True,
                        show_controls = False,
                        show_data = False,
                        dataCB = self.colorImageConnectCb,
                        msg_if = self.msg_if)

        self.navpose_if = ConnectNavPoseIF(
                        connect_name = NAVPOSE_CONNECT_NAME,
                        show_selector = True,
                        show_controls = False,
                        show_data = False,
                        dataCB = self.navposeConnectCb,
                        msg_if = self.msg_if)


    ###################
    ## Connect IF Callbacks
    #
    # Each connect IF invokes its dataCB with a single data dict. The
    # callback stores that dict and the IF's current status message (via
    # get_status_msg()) on EVERY invocation, so the rest of the app reads the most
    # recent values from the class attributes rather than re-querying the IF. On
    # the FIRST invocation per IF it also logs both, then sets the got_first flag
    # so it logs only once. Unlike the wpilib app's callbacks, these also
    # republish status when the selected source topics change: this app reports
    # those topics in its own status message, so the RUI must see a new selection
    # without waiting out a full status timer period, and the callbacks run at the
    # source's data rate rather than the 1 Hz status rate. The stored data itself
    # is not consumed here -- this app wires sources, obstacle detection
    # processing is separate work.
    #
    # NOTE: the cached status msgs are for this app's own use. publish_status()
    # deliberately does NOT read them -- see getDepthMapImageTopic().

    def depthMapConnectCb(self, data_dict):
        self.depth_map_dict = data_dict
        self.depth_map_status = self.depth_map_if.get_status_msg()
        if self.got_first_depth_map is False:
            self.got_first_depth_map = True
            self.msg_if.pub_info("Depth map first-connection data dict: " + str(self.depth_map_dict))
            self.msg_if.pub_info("Depth map first-connection status message: " + str(self.depth_map_status))
        self.publishStatusIfSourcesChanged()

    def colorImageConnectCb(self, data_dict):
        self.color_image_dict = data_dict
        self.color_image_status = self.color_image_if.get_status_msg()
        if self.got_first_color_image is False:
            self.got_first_color_image = True
            self.msg_if.pub_info("Color image first-connection data dict: " + str(self.color_image_dict))
            self.msg_if.pub_info("Color image first-connection status message: " + str(self.color_image_status))
        self.publishStatusIfSourcesChanged()

    def navposeConnectCb(self, data_dict):
        self.navpose_dict = data_dict
        self.navpose_status = self.navpose_if.get_status_msg()
        if self.got_first_navpose is False:
            self.got_first_navpose = True
            self.msg_if.pub_info("NavPose first-connection data dict: " + str(self.navpose_dict))
            self.msg_if.pub_info("NavPose first-connection status message: " + str(self.navpose_status))
        self.publishStatusIfSourcesChanged()


    ###################
    ## Status Publishers

    def statusPublishCb(self, timer):
        self.publish_status()

    # Depth map image topic belonging to the SELECTED depth map, or 'None'.
    #
    # The operator never selects a depth map image, so this is derived. It is read
    # LIVE off the connect IF rather than from the cached self.depth_map_status:
    # that cache only refreshes when a depth map frame arrives, so after a switch
    # to a depth map with no image -- or a deselect -- it would keep reporting the
    # previous selection's image topic. ConnectDepthMapIF clears its own status_msg
    # on resubscribe, on unsubscribe, and after its connection timeout, so the live
    # read is the one that tracks the selection.
    #
    # Ownership is checked before the status is trusted at all. ConnectDepthMapIF
    # resubscribes on its own 1 Hz tick, so for up to a second after a selection
    # change get_status_msg() still returns the PREVIOUS depth map's status;
    # DepthMapIF reports its own namespace in DepthMapStatus.depth_map_topic, so a
    # status whose depth_map_topic is not the selected topic is discarded.
    #
    # DepthMapStatus.image_topic is only this depth map's own image when it
    # resolves UNDER the selected depth map namespace. On every DepthMapIF producer
    # in nepi_api today it does not: DepthMapIF._updaterCb fills image_topic with
    # the SIBLING <device>/color_image, so taking it at face value would put the
    # color image in the depth map viewer. Anything not under the selection is
    # therefore rejected in favor of the topic DepthMapIF actually publishes its
    # depth map image on -- <depth_map>/depth_map_image, gated on the producer's
    # own DepthMapStatus.img_pub_enabled flag. A depth map that reports no image
    # yields 'None' and the RUI viewer stays unmounted; the color image is never
    # substituted.
    def getDepthMapImageTopic(self):
        if self.depth_map_if is None:
            return 'None'
        selected_topic = self.depth_map_if.get_selected_topic()
        if selected_topic is None or selected_topic == '' or selected_topic == 'None':
            return 'None'
        status_msg = self.depth_map_if.get_status_msg()
        if status_msg is None:
            return 'None'
        if status_msg.depth_map_topic != selected_topic:
            return 'None'
        image_topic = status_msg.image_topic
        if image_topic is not None and image_topic.startswith(selected_topic + '/'):
            return image_topic
        if status_msg.img_pub_enabled == True:
            return nepi_sdk.create_namespace(selected_topic, DEPTH_MAP_IMAGE_SUBTOPIC)
        return 'None'

    # Source topics this app reports, keyed by status message field name. A
    # connect IF that is not built yet reports 'None', which is also what
    # get_selected_topic() returns for an unselected source.
    def getSourceTopics(self):
        source_topics = {
            'depth_map_topic': 'None',
            'depth_map_image_topic': self.getDepthMapImageTopic(),
            'image_topic': 'None',
            'navpose_topic': 'None'
        }
        if self.depth_map_if is not None:
            source_topics['depth_map_topic'] = self.depth_map_if.get_selected_topic()
        if self.color_image_if is not None:
            source_topics['image_topic'] = self.color_image_if.get_selected_topic()
        if self.navpose_if is not None:
            source_topics['navpose_topic'] = self.navpose_if.get_selected_topic()
        return source_topics

    def publishStatusIfSourcesChanged(self):
        if self.getSourceTopics() != self.last_source_topics:
            self.publish_status()

    def publish_status(self):
        status_msg = NepiAppObstaclesStatus()
        status_msg.enabled = self.enabled
        status_msg.options = self.options
        status_msg.selected_option = self.selected_option
        status_msg.value = self.value
        source_topics = self.getSourceTopics()
        status_msg.depth_map_topic = source_topics['depth_map_topic']
        status_msg.depth_map_image_topic = source_topics['depth_map_image_topic']
        status_msg.image_topic = source_topics['image_topic']
        status_msg.navpose_topic = source_topics['navpose_topic']
        self.last_source_topics = source_topics
        if self.node_if is not None:
            self.node_if.publish_pub('status_pub', status_msg)


    #######################
    # Utility Functions
    #######################

    def cleanup_actions(self):
        self.msg_if.pub_info("OBSTACLES: Shutting down: Executing script cleanup actions")
        if self.depth_map_if is not None:
            self.depth_map_if.unregister()
        if self.color_image_if is not None:
            self.color_image_if.unregister()
        if self.navpose_if is not None:
            self.navpose_if.unregister()


#########################################
# Main
#########################################
if __name__ == '__main__':
    NepiObstaclesApp()
