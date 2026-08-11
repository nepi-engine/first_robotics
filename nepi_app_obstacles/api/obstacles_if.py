#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi engine (nepi_engine) repo
# (see https://github.com/nepi-engine/nepi_engine)
#
# License: NEPI Engine repo source-code and NEPI Images that use this source-code
# are licensed under the "Numurus Software License",
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
import threading

from std_msgs.msg import Float32, Bool, String
from sensor_msgs.msg import Image

from nepi_interfaces.msg import DepthMapStatus
from nepi_interfaces.msg import MgrSystemStatus
from nepi_interfaces.msg import StringArray
from nepi_interfaces.msg import ProcessStatus
from nepi_interfaces.msg import NavPose

from nepi_app_obstacles.msg import Obstacle, Obstacles, ObstaclesDepthMap, ObstaclesStatus

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_system
from nepi_sdk import nepi_img
from nepi_sdk import nepi_nav

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeSubscribersIF, NodeClassIF
from nepi_api.system_if import ControlsIF, SaveDataIF, StatesIF, TriggersIF


SYSTEM_ALL_TOPIC = 'all'

#########################################
# Obstacles Node IF
#########################################
OBSTACLES_TOPIC = 'obstacles'

# The depth map segmentation ships on its own topic rather than inside the
# Obstacles message, so a consumer that only wants the obstacle list is not
# forced to carry two full-size range images per cycle. It is published on this
# process's own namespace only -- not on the collective 'all' namespace the
# obstacle list fans out on -- because the only consumer is this process's own
# image pub node and an all-namespace copy would double the image bandwidth for
# nothing.
OBSTACLES_DEPTH_MAP_TOPIC = 'obstacles_depth_map'

# Source topics are depth maps. They are discovered by their DepthMapStatus
# publisher rather than by string-matching an Image topic name. Matching on
# 'depth_map' as a substring would also match 'depth_map_image', which is a
# rendered colour image, not range data.
SOURCE_STATUS_MSG_NAME = 'DepthMapStatus'

MIN_MAX_RATE = 1
MAX_MAX_RATE = 20
DEFAULT_MAX_PROC_RATE = 10
DEFAULT_MAX_IMG_RATE = 10
DEFAULT_USE_LAST_IMAGE = True

GET_IMAGE_TIMEOUT_SEC = 1

CONNECTED_TIMEOUT_SEC = 2

# Node file for the overlay image publisher this IF launches. It installs into
# the app package's lib folder via catkin_install_python, which sits beside the
# nepi_api lib folder reported by nepi_system.get_system_folders().
IMG_PUB_PKG_NAME = 'nepi_app_obstacles'
IMG_PUB_NODE_FILE = 'obstacles_app_img_pub_node.py'

# Image data products the image pub node publishes, one set per active source.
# Named after the platform convention DepthMapIF sets -- raw range data at
# <name>, its colourized viewable render at <name>_image. These must match the
# names in obstacles_app_img_pub_node.py; that node owns the publishers, this
# class only reports where they will appear.
OBSTACLES_IMG_PRODUCT = 'obstacles_image'
GROUND_MAP_IMG_PRODUCT = 'ground_depth_map_image'
OBSTACLES_MAP_IMG_PRODUCT = 'obstacles_depth_map_image'


class ObstaclesIF:

    OBSTACLES_DATA_PRODUCTS = ['obstacles',
                               OBSTACLES_IMG_PRODUCT,
                               GROUND_MAP_IMG_PRODUCT,
                               OBSTACLES_MAP_IMG_PRODUCT]

    # An obstacles process must never consume its own overlay output as an
    # input source; skip these product basenames even if a stale or explicit
    # selection lists them.
    OUTPUT_IMG_PRODUCTS = [OBSTACLES_IMG_PRODUCT,
                           GROUND_MAP_IMG_PRODUCT,
                           OBSTACLES_MAP_IMG_PRODUCT]

    namespace = '~'
    obstacles_namespace = '~'
    all_namespace = None
    all_obstacles_namespace = None

    process_name = 'obstacles'

    process_status_msg = ProcessStatus()
    obstacles_status_msg = ObstaclesStatus()

    node_if = None
    controls_if = None
    save_data_if = None
    states_if = None
    triggers_if = None
    save_data_namespace = 'None'

    data_products = OBSTACLES_DATA_PRODUCTS

    available_source_topics = []

    api_lib_folder = '/opt/nepi/nepi_engine/lib/nepi_api'
    img_pub_lib_folder = '/opt/nepi/nepi_engine/lib/nepi_app_obstacles'

    source_ifs_dict = dict()
    source_ifs_lock = threading.Lock()
    sources_info_dict = dict()

    source_dict = dict()

    navpose_dict = dict()
    navpose_dict_lock = threading.Lock()

    msg_str = 'Loading'
    active_source_topics = []
    cur_source_topic = "None"

    # The active source order getProcessStatus last used to build
    # imaging_source_topics / imaging_pub_topics. publish_status maps the two
    # segmentation topic lists over this same list rather than re-walking
    # sources_info_dict, so all four lists stay index-aligned even if a source
    # is added or purged between the two calls.
    imaging_source_topics = []

    get_source_topic = "None"
    got_source_topic = None

    data_dict = dict()
    controls_dict = dict()

    first_process_complete = False
    processing_state = False

    source_receive_latencies = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    source_receive_rates = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    preprocess_times = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    preprocess_latencies = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    preprocess_rates = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    process_times = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    process_latencies = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    process_rates = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    is_processing = False
    process_state = False
    last_receive_source_time = nepi_sdk.get_time()
    last_detect_time = nepi_sdk.get_time()
    last_process_time = nepi_sdk.get_time()

    enabled = True
    max_process_rate_hz = DEFAULT_MAX_PROC_RATE
    max_image_pub_rate_hz = DEFAULT_MAX_IMG_RATE
    use_last_image = DEFAULT_USE_LAST_IMAGE

    auto_select_enabled = True
    auto_select_active = True
    selected_sources = []

    imaging_enabled = True
    launch_node_process = None
    pub_img_node_name = ""
    pub_img_namespace = ""

    next_source_topic = "None"

    node_if_prefix = 'obstacles_'

    active_nodes = []
    active_topics = []
    active_topic_types = []
    active_services = []

    save_config_enabled = True

    min_range_m = 0.0
    max_range_m = 100.0

    has_navpose = False
    navpose_topic = ''
    navpose_topic_connected = False

    has_color_image = False
    color_image_topic = ''
    color_image_topic_connected = False

    image_controls_dict = dict(
        full_screen_enabled = False,
        show_sources_enabled = True,
        show_ground_enabled = True,
        show_obstacles_enabled = True
    )

    ready = False

    def __init__(self,
                namespace,
                description,
                data_dict,
                controls_dict,
                processResultsFunction,
                enable_image_pub = True,
                log_name = None,
                log_name_list = [],
                msg_if = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################
        # Create Msg Class
        if msg_if is not None:
            self.msg_if = msg_if
        else:
            self.msg_if = MsgIF()
        self.log_name_list = copy.deepcopy(log_name_list)
        self.log_name_list.append(self.class_name)
        if log_name is not None:
            self.log_name_list.append(log_name)
        self.msg_if.pub_debug("Starting Obstacles IF Initialization Processes", log_name_list = self.log_name_list)

        ##############################
        # Init Class Variables

        if namespace is None:
            namespace = self.node_namespace
        self.namespace = nepi_sdk.get_full_namespace(namespace)
        self.obstacles_namespace = nepi_sdk.create_namespace(self.namespace, OBSTACLES_TOPIC)

        # Collective controls publish on the shared namespaces, which fans a
        # single command out to every obstacles process on the device.
        self.all_namespace = nepi_sdk.create_namespace(self.base_namespace, SYSTEM_ALL_TOPIC)
        self.all_obstacles_namespace = nepi_sdk.create_namespace(self.all_namespace, OBSTACLES_TOPIC)

        self.enable_image_pub = enable_image_pub
        self.description = description
        self.processResults = processResultsFunction

        ##############################
        # Get System Folders
        self.msg_if.pub_info("Waiting for system folders", log_name_list = self.log_name_list)
        system_folders = nepi_system.get_system_folders(log_name_list = [self.node_name])
        while system_folders is None and nepi_sdk.is_shutdown() == False:
            system_folders = nepi_system.get_system_folders(log_name_list = [self.node_name])
            nepi_sdk.sleep(1)

        if system_folders is not None and 'api_lib' in system_folders.keys():
            self.api_lib_folder = system_folders['api_lib']
        # The app's own lib folder is a sibling of the nepi_api lib folder.
        self.img_pub_lib_folder = os.path.join(os.path.dirname(self.api_lib_folder), IMG_PUB_PKG_NAME)
        self.msg_if.pub_info("Using image pub lib folder: " + str(self.img_pub_lib_folder), log_name_list = self.log_name_list)

        ## Init Status Messages
        self.process_status_msg.node_name = self.node_name
        self.process_status_msg.namespace = self.obstacles_namespace

        ##############################
        ### Setup Node

        # Configs Dict ########################
        self.CONFIGS_DICT = {
                'init_callback': self.initCb,
                'reset_callback': self.resetCb,
                'factory_reset_callback': self.factoryResetCb,
                'init_configs': True,
                'namespace':  self.namespace,
        }

        # Params Config Dict ####################
        # NOTE: for params the registry key IS the wire name
        # (namespace + key), so the obstacles_ prefix is deliberate and
        # load-bearing, not just collision insurance.
        self.PARAMS_DICT = {
            self.node_if_prefix + 'enabled': {
                'namespace': self.namespace,
                'factory_val': self.enabled
            },
            self.node_if_prefix + 'auto_select_enabled': {
                'namespace': self.namespace,
                'factory_val': self.auto_select_enabled
            },
            self.node_if_prefix + 'selected_sources': {
                'namespace': self.namespace,
                'factory_val': []
            },
            self.node_if_prefix + 'max_process_rate_hz': {
                'namespace': self.namespace,
                'factory_val': DEFAULT_MAX_PROC_RATE
            },
            self.node_if_prefix + 'max_image_pub_rate_hz': {
                'namespace': self.namespace,
                'factory_val': DEFAULT_MAX_IMG_RATE
            },
            self.node_if_prefix + 'use_last_image': {
                'namespace': self.namespace,
                'factory_val': DEFAULT_USE_LAST_IMAGE
            },
            self.node_if_prefix + 'imaging_enabled': {
                'namespace': self.namespace,
                'factory_val': self.imaging_enabled
            },
            self.node_if_prefix + 'image_controls': {
                'namespace': self.namespace,
                'factory_val': self.image_controls_dict
            },
        }

        # Services Config Dict ####################
        self.SRVS_DICT = None

        # Pubs Config Dict ####################
        self.PUBS_DICT = {
            'obstacles_status_pub': {
                'msg': ObstaclesStatus,
                'namespace': self.obstacles_namespace,
                'topic': 'status',
                'qsize': 1,
                'latch': True
            },
            'obstacles_pub': {
                'msg': Obstacles,
                'namespace': self.namespace,
                'topic': OBSTACLES_TOPIC,
                'qsize': 1,
                'latch': False
            },
            'obstacles_all_pub': {
                'msg': Obstacles,
                'namespace': self.all_namespace,
                'topic': OBSTACLES_TOPIC,
                'qsize': 1,
                'latch': False
            },
            'obstacles_depth_map_pub': {
                'msg': ObstaclesDepthMap,
                'namespace': self.namespace,
                'topic': OBSTACLES_DEPTH_MAP_TOPIC,
                'qsize': 1,
                'latch': False
            },
        }

        # Subs Config Dict ####################
        self.SUBS_DICT = {
            ############
            # Obstacles
            ############
            self.node_if_prefix + 'enable': {
                'namespace': self.obstacles_namespace,
                'topic': 'enable',
                'msg': Bool,
                'qsize': 10,
                'callback': self.setEnableCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'set_auto_select_enable': {
                'namespace': self.obstacles_namespace,
                'topic': 'set_auto_select_enable',
                'msg': Bool,
                'qsize': 10,
                'callback': self.setAutoSelectEnableCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'set_source_topic': {
                'namespace': self.obstacles_namespace,
                'topic': 'set_source_topic',
                'msg': String,
                'qsize': 10,
                'callback': self.setSourceTopicCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'set_source_topics': {
                'namespace': self.obstacles_namespace,
                'topic': 'set_source_topics',
                'msg': StringArray,
                'qsize': 10,
                'callback': self.setSourceTopicsCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'add_source_topic': {
                'namespace': self.obstacles_namespace,
                'topic': 'add_source_topic',
                'msg': String,
                'qsize': 10,
                'callback': self.addSourceTopicCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'add_source_topics': {
                'namespace': self.obstacles_namespace,
                'topic': 'add_source_topics',
                'msg': StringArray,
                'qsize': 10,
                'callback': self.addSourceTopicsCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'remove_source_topic': {
                'namespace': self.obstacles_namespace,
                'topic': 'remove_source_topic',
                'msg': String,
                'qsize': 10,
                'callback': self.removeSourceTopicCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'remove_source_topics': {
                'namespace': self.obstacles_namespace,
                'topic': 'remove_source_topics',
                'msg': StringArray,
                'qsize': 10,
                'callback': self.removeSourceTopicsCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'set_image_pub': {
                'namespace': self.obstacles_namespace,
                'topic': 'set_image_pub',
                'msg': Bool,
                'qsize': 10,
                'callback': self.setPubImageCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'set_max_process_rate': {
                'namespace': self.obstacles_namespace,
                'topic': 'set_max_process_rate',
                'msg': Float32,
                'qsize': 10,
                'callback': self.setMaxProcRateCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'set_max_image_pub_rate': {
                'namespace': self.obstacles_namespace,
                'topic': 'set_max_image_pub_rate',
                'msg': Float32,
                'qsize': 10,
                'callback': self.setMaxImgRateCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'set_use_last_image': {
                'namespace': self.obstacles_namespace,
                'topic': 'set_use_last_image',
                'msg': Bool,
                'qsize': 10,
                'callback': self.setUseLastImageCb,
                'callback_args': ()
            },
            ############
            # Image Overlay Controls
            ############
            self.node_if_prefix + 'set_full_screen': {
                'namespace': self.obstacles_namespace,
                'topic': 'set_full_screen',
                'msg': Bool,
                'qsize': 10,
                'callback': self.setFullScreenCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'set_show_sources': {
                'namespace': self.obstacles_namespace,
                'topic': 'set_show_sources',
                'msg': Bool,
                'qsize': 10,
                'callback': self.setShowSourcesCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'set_show_ground': {
                'namespace': self.obstacles_namespace,
                'topic': 'set_show_ground',
                'msg': Bool,
                'qsize': 10,
                'callback': self.setShowGroundCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'set_show_obstacles': {
                'namespace': self.obstacles_namespace,
                'topic': 'set_show_obstacles',
                'msg': Bool,
                'qsize': 10,
                'callback': self.setShowObstaclesCb,
                'callback_args': ()
            },
            ############
            # All Obstacles
            ############
            self.node_if_prefix + 'all_enable': {
                'namespace': self.all_obstacles_namespace,
                'topic': 'enable',
                'msg': Bool,
                'qsize': 10,
                'callback': self.setEnableCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'all_set_source_topic': {
                'namespace': self.all_obstacles_namespace,
                'topic': 'set_source_topic',
                'msg': String,
                'qsize': 10,
                'callback': self.setSourceTopicCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'all_set_source_topics': {
                'namespace': self.all_obstacles_namespace,
                'topic': 'set_source_topics',
                'msg': StringArray,
                'qsize': 10,
                'callback': self.setSourceTopicsCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'all_add_source_topic': {
                'namespace': self.all_obstacles_namespace,
                'topic': 'add_source_topic',
                'msg': String,
                'qsize': 10,
                'callback': self.addSourceTopicCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'all_add_source_topics': {
                'namespace': self.all_obstacles_namespace,
                'topic': 'add_source_topics',
                'msg': StringArray,
                'qsize': 10,
                'callback': self.addSourceTopicsCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'all_remove_source_topic': {
                'namespace': self.all_obstacles_namespace,
                'topic': 'remove_source_topic',
                'msg': String,
                'qsize': 10,
                'callback': self.removeSourceTopicCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'all_remove_source_topics': {
                'namespace': self.all_obstacles_namespace,
                'topic': 'remove_source_topics',
                'msg': StringArray,
                'qsize': 10,
                'callback': self.removeSourceTopicsCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'all_set_image_pub': {
                'namespace': self.all_obstacles_namespace,
                'topic': 'set_image_pub',
                'msg': Bool,
                'qsize': 10,
                'callback': self.setPubImageCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'all_set_max_process_rate': {
                'namespace': self.all_obstacles_namespace,
                'topic': 'set_max_process_rate',
                'msg': Float32,
                'qsize': 10,
                'callback': self.setMaxProcRateCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'all_set_max_image_pub_rate': {
                'namespace': self.all_obstacles_namespace,
                'topic': 'set_max_image_pub_rate',
                'msg': Float32,
                'qsize': 10,
                'callback': self.setMaxImgRateCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'all_set_use_last_image': {
                'namespace': self.all_obstacles_namespace,
                'topic': 'set_use_last_image',
                'msg': Bool,
                'qsize': 10,
                'callback': self.setUseLastImageCb,
                'callback_args': ()
            },
            ############
            # Misc
            ############
            self.node_if_prefix + 'system_status': {
                'msg': MgrSystemStatus,
                'namespace': self.base_namespace,
                'topic': 'status',
                'qsize': 5,
                'callback': self.systemStatusCb
            },
        }

        # Create Node Class ####################
        self.node_if = NodeClassIF(
                        configs_dict = self.CONFIGS_DICT,
                        params_dict = self.PARAMS_DICT,
                        services_dict = self.SRVS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        log_name_list = self.log_name_list,
                        msg_if = self.msg_if
                        )
        self.node_if.wait_for_ready()

        self.initCb(do_updates = True)

        ###############################
        # Create System IFs

        # Setup Controls IF. node_if is left as None so the IF builds and owns
        # its own NodeClassIF -- the current convention, and what keeps this
        # class's registry keys from ever colliding with the sub-IF's.
        self.controls_dict = copy.deepcopy(controls_dict)
        self.controls_if = ControlsIF(
                        controls_name = 'controls',
                        controls_display_name = 'Obstacles Controls',
                        controls_description = self.description,
                        controls_init_dict = self.controls_dict,
                        show_controls = True,
                        has_show_control = False,
                        log_name = 'controls',
                        log_name_list = self.log_name_list,
                        msg_if = self.msg_if)
        self.controls_if.wait_for_controls_ready()

        self.data_dict = copy.deepcopy(data_dict)

        # Setup States IF
        self.states_if = StatesIF(
                        states_name = 'obstacles',
                        get_states_dict_function = self.getObstaclesStates,
                        log_name_list = self.log_name_list,
                        msg_if = self.msg_if)

        # Setup Triggers IF
        self.triggers_dict = {
                        "obstacles_trigger": {
                            "name": "obstacles_trigger",
                            "node_name": self.node_name,
                            "description": "Triggered on obstacle detection",
                            "data_str_list": ["None"],
                            "time": nepi_utils.get_time()
                            }
        }
        self.triggers_if = TriggersIF(triggers_dict = self.triggers_dict,
                        log_name_list = self.log_name_list,
                        msg_if = self.msg_if)

        # Setup Save Data IF
        factory_data_rates = {}
        for d in self.data_products:
            factory_data_rates[d] = [1.0, 0.0, 100]

        self.save_data_if = SaveDataIF(namespace = self.namespace,
                        data_products = self.data_products,
                        factory_rate_dict = factory_data_rates,
                        log_name_list = self.log_name_list,
                        msg_if = self.msg_if)
        nepi_sdk.sleep(1)
        if self.save_data_if is not None:
            self.save_data_namespace = self.save_data_if.get_namespace()
            self.process_status_msg.save_data_topic = self.save_data_namespace
            self.msg_if.pub_info("Using save_data namespace: " + str(self.save_data_namespace), log_name_list = self.log_name_list)

        ###############################
        # Launch the overlay image publisher node
        self.launch_image_pub_node()

        ##########################
        # Complete Initialization

        # Start Timer Processes
        nepi_sdk.start_timer_process((1.0), self.publishStatusCb)
        nepi_sdk.start_timer_process((0.1), self.updaterCb, oneshot = True)
        nepi_sdk.start_timer_process((0.1), self.updateNextTopicCb, oneshot = True)
        nepi_sdk.start_timer_process((1.0), self.processObstaclesCb, oneshot = True)

        self.msg_str = 'Loaded'
        self.ready = True
        ##########################
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ##########################


    ###############################
    # Class Public Methods
    ###############################

    def get_ready_state(self):
        """Return the ready state of the interface.

        Returns:
            bool: True once initialization has completed, False otherwise.
        """
        return self.ready

    def get_namespace(self):
        """Return the fully-resolved namespace this obstacles process owns.

        Returns:
            str: The ``<node>/obstacles`` namespace carrying the status topic
                and every command topic.
        """
        return self.obstacles_namespace

    def get_controls_dict(self):
        """Return a copy of the live controls dictionary.

        Returns:
            dict: The nepi_controls controls dictionary held by this
                interface's ControlsIF, or an empty dict if the ControlsIF is
                not yet built.
        """
        if self.controls_if is None:
            return dict()
        return copy.deepcopy(self.controls_if.get_controls_dict())

    def launch_image_pub_node(self):
        """Launch the obstacles overlay image publisher node as a subprocess.

        Resolves the image publisher node file path in the app's lib folder and
        starts it in the parent namespace so its own node namespace is
        ``<app namespace>_img_pub``. Does nothing if the node is already
        running, if the node file cannot be found, or if image publishing was
        disabled at construction.
        """
        node_name = self.node_name + "_img_pub"
        launch_namespace = os.path.dirname(self.namespace)
        node_namespace = self.namespace + "_img_pub"

        node_file_path = os.path.join(self.img_pub_lib_folder, IMG_PUB_NODE_FILE)
        if self.launch_node_process is not None:
            self.msg_if.pub_warn("Node Already Launched: " + node_name, log_name_list = self.log_name_list)
        elif self.enable_image_pub == False:
            self.msg_if.pub_info("Image pub node disabled", log_name_list = self.log_name_list)
        elif os.path.exists(node_file_path) == False:
            self.msg_if.pub_warn("Could not find Node File at: " + node_file_path, log_name_list = self.log_name_list)
        else:
            self.msg_if.pub_info("Launching Obstacles Img Node with settings " + str([IMG_PUB_PKG_NAME, IMG_PUB_NODE_FILE, node_name]), log_name_list = self.log_name_list)
            [success, msg, sub_process] = nepi_sdk.launch_node(IMG_PUB_PKG_NAME, IMG_PUB_NODE_FILE, node_name, namespace = launch_namespace)
            if success == True:
                self.launch_node_process = sub_process
                self.pub_img_node_name = node_name
                self.pub_img_namespace = node_namespace
            self.msg_if.pub_info("Node launch return msg: " + str(msg), log_name_list = self.log_name_list)

    def kill_image_pub_node(self):
        """Terminate the running obstacles overlay image publisher node.

        Sends a kill signal to the subprocess started by
        ``launch_image_pub_node`` and clears the process handle and node name
        on success. Logs a warning if the node is not currently running.
        """
        if self.launch_node_process is None:
            self.msg_if.pub_warn("Node Not Running", log_name_list = self.log_name_list)
        else:
            success = nepi_sdk.kill_node_process(self.pub_img_node_name, self.launch_node_process)
            if success == True:
                self.launch_node_process = None
                self.pub_img_node_name = ""
                self.pub_img_namespace = ""
                self.msg_if.pub_info("Node Killed", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_warn("Failed to Kill Node", log_name_list = self.log_name_list)

    def set_pub_image(self, enable):
        """Enable or disable overlay image publishing and persist the setting.

        Args:
            enable (bool): True to enable obstacles image publishing,
                False to disable it.
        """
        self.imaging_enabled = enable
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix + 'imaging_enabled', self.imaging_enabled)
            self.save_config()

    def save_config(self):
        """Persist this interface's parameters through the config manager."""
        if self.save_config_enabled == True:
            if self.node_if is not None:
                self.node_if.save_config()

    def publish_status(self):
        """Assemble and publish the obstacles status message.

        Populates the embedded ProcessStatus and every obstacles-specific
        field from current internal state, then publishes on
        ``<node>/obstacles/status``.
        """
        process_status_msg = self.getProcessStatus()

        self.obstacles_status_msg.process_status = process_status_msg

        self.obstacles_status_msg.controls_topic = '' if self.controls_if is None else self.controls_if.get_namespace()

        self.obstacles_status_msg.min_range_m = float(self.min_range_m)
        self.obstacles_status_msg.max_range_m = float(self.max_range_m)

        self.obstacles_status_msg.has_navpose = self.has_navpose
        self.obstacles_status_msg.navpose_topic = self.navpose_topic
        self.obstacles_status_msg.navpose_topic_connected = self.navpose_topic_connected

        self.obstacles_status_msg.has_color_image = self.has_color_image
        self.obstacles_status_msg.color_image_topic = self.color_image_topic
        self.obstacles_status_msg.color_image_topic_connected = self.color_image_topic_connected

        # Advertise the two segmentation render topics the same way
        # process_status.imaging_pub_topics advertises the overlay one, walking
        # the active source list getProcessStatus just built so index i of all
        # four lists names the same source.
        sources_info_dict = copy.deepcopy(self.sources_info_dict)
        ground_image_pub_topics = []
        obstacles_image_pub_topics = []
        for source_topic in self.imaging_source_topics:
            # An empty entry rather than a skipped one: dropping an element
            # would shift every later index out of step with the process_status
            # lists, which is the one thing these lists promise.
            info_dict = sources_info_dict.get(source_topic, dict())
            ground_image_pub_topics.append(info_dict.get('ground_img_pub_topic', ''))
            obstacles_image_pub_topics.append(info_dict.get('obstacles_img_pub_topic', ''))
        self.obstacles_status_msg.ground_image_pub_topics = ground_image_pub_topics
        self.obstacles_status_msg.obstacles_image_pub_topics = obstacles_image_pub_topics

        self.obstacles_status_msg.full_screen_enabled = self.image_controls_dict['full_screen_enabled']
        self.obstacles_status_msg.show_sources_enabled = self.image_controls_dict['show_sources_enabled']
        self.obstacles_status_msg.show_ground_enabled = self.image_controls_dict['show_ground_enabled']
        self.obstacles_status_msg.show_obstacles_enabled = self.image_controls_dict['show_obstacles_enabled']

        if self.node_if is not None:
            self.node_if.publish_pub('obstacles_status_pub', self.obstacles_status_msg)

    def unregister(self):
        """Tear down the image pub node and this interface's ROS registrations."""
        self.kill_image_pub_node()
        if self.node_if is not None:
            self.node_if.unregister_class()

    ###############################
    # Class Private Methods
    ###############################

    def systemStatusCb(self, msg):
        self.active_nodes = msg.active_nodes
        self.active_topics = msg.active_topics
        self.active_topic_types = msg.active_topic_types
        self.active_services = msg.active_services

    def getObstaclesStates(self):
        states_dict = dict()
        states_dict['running'] = self.enabled
        states_dict['obstacles'] = self.processing_state
        self.processing_state = False
        return states_dict

    def initCb(self, do_updates = False):
        if self.node_if is not None:
            self.imaging_enabled = self.node_if.get_param(self.node_if_prefix + 'imaging_enabled')
            self.enabled = self.node_if.get_param(self.node_if_prefix + 'enabled')
            self.max_process_rate_hz = self.node_if.get_param(self.node_if_prefix + 'max_process_rate_hz')
            self.max_image_pub_rate_hz = self.node_if.get_param(self.node_if_prefix + 'max_image_pub_rate_hz')
            self.use_last_image = self.node_if.get_param(self.node_if_prefix + 'use_last_image')

            self.selected_sources = self.node_if.get_param(self.node_if_prefix + 'selected_sources')
            self.auto_select_enabled = self.node_if.get_param(self.node_if_prefix + 'auto_select_enabled')
            self.auto_select_active = self.auto_select_enabled

            image_controls_dict = self.node_if.get_param(self.node_if_prefix + 'image_controls')
            if isinstance(image_controls_dict, dict):
                for key in self.image_controls_dict.keys():
                    if key in image_controls_dict.keys():
                        self.image_controls_dict[key] = image_controls_dict[key]

            self.msg_if.pub_info("Init selected sources: " + str(self.selected_sources), log_name_list = self.log_name_list)
        if do_updates == True:
            pass
        self.publish_status()

    def resetCb(self, do_updates = True):
        if self.controls_if is not None:
            self.controls_if.reset()
        self.initCb(do_updates = do_updates)

    def factoryResetCb(self, do_updates = True):
        if self.controls_if is not None:
            self.controls_if.factory_reset()
        self.initCb(do_updates = do_updates)

    ##########################################
    # Command Callbacks

    def setEnableCb(self, msg):
        self.setEnable(msg.data)

    def setEnable(self, enabled, save_config = True):
        last_val = copy.deepcopy(self.enabled)
        self.enabled = enabled
        self.publish_status()
        if self.node_if is not None and enabled != last_val and save_config == True:
            self.node_if.set_param(self.node_if_prefix + 'enabled', self.enabled)
            self.save_config()
        if enabled == False and not nepi_sdk.is_shutdown():
            self.next_source_topic = "None"

    def setAutoSelectEnableCb(self, msg):
        self.setAutoSelectEnable(msg.data)

    def setAutoSelectEnable(self, enabled):
        self.selected_sources = []
        self.auto_select_active = enabled
        self.auto_select_enabled = enabled
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix + 'auto_select_enabled', self.auto_select_enabled)
            self.save_config()

    def setSourceTopicCb(self, msg):
        self.setSourceTopic(msg.data)

    def setSourceTopic(self, source_topic, save_config = True):
        self.selected_sources = [source_topic]
        self.auto_select_active = False
        self.publish_status()
        if self.node_if is not None and save_config == True:
            self.node_if.set_param(self.node_if_prefix + 'selected_sources', self.selected_sources)
            self.save_config()

    def setSourceTopicsCb(self, msg):
        self.setSourceTopics(list(msg.array))

    def setSourceTopics(self, source_topics, save_config = True):
        self.selected_sources = source_topics
        self.auto_select_active = False
        self.publish_status()
        if self.node_if is not None and save_config == True:
            self.node_if.set_param(self.node_if_prefix + 'selected_sources', self.selected_sources)
            self.save_config()

    def addSourceTopicCb(self, msg):
        self.addSourceTopic(msg.data)

    def addSourceTopicsCb(self, msg):
        for source_topic in msg.array:
            self.addSourceTopic(source_topic)

    def addSourceTopic(self, source_topic):
        source_topics = copy.deepcopy(self.selected_sources)
        if source_topic not in source_topics:
            source_topics.append(source_topic)
        else:
            self.msg_if.pub_warn('Source topic already selected: ' + str(source_topic), log_name_list = self.log_name_list)
        self.selected_sources = source_topics
        self.auto_select_active = False
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix + 'selected_sources', self.selected_sources)
            self.save_config()

    def removeSourceTopicCb(self, msg):
        self.removeSourceTopic(msg.data)

    def removeSourceTopicsCb(self, msg):
        for source_topic in msg.array:
            self.removeSourceTopic(source_topic)

    def removeSourceTopic(self, source_topic, save_config = True):
        source_topics = copy.deepcopy(self.selected_sources)
        if source_topic in source_topics:
            source_topics.remove(source_topic)
        self.selected_sources = source_topics
        self.auto_select_active = False
        self.publish_status()
        if self.node_if is not None and save_config == True:
            self.node_if.set_param(self.node_if_prefix + 'selected_sources', self.selected_sources)
            self.save_config()

    def setMaxProcRateCb(self, msg):
        max_rate = msg.data
        if max_rate < MIN_MAX_RATE:
            max_rate = MIN_MAX_RATE
        elif max_rate > MAX_MAX_RATE:
            max_rate = MAX_MAX_RATE
        self.max_process_rate_hz = max_rate
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix + 'max_process_rate_hz', self.max_process_rate_hz)
            self.save_config()

    def setMaxImgRateCb(self, msg):
        max_rate = msg.data
        if max_rate < MIN_MAX_RATE:
            max_rate = MIN_MAX_RATE
        elif max_rate > MAX_MAX_RATE:
            max_rate = MAX_MAX_RATE
        self.max_image_pub_rate_hz = max_rate
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix + 'max_image_pub_rate_hz', self.max_image_pub_rate_hz)
            self.save_config()

    def setUseLastImageCb(self, msg):
        self.use_last_image = msg.data
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix + 'use_last_image', self.use_last_image)
            self.save_config()

    def setPubImageCb(self, msg):
        self.set_pub_image(msg.data)

    def setFullScreenCb(self, msg):
        self.setImageControl('full_screen_enabled', msg.data)

    def setShowSourcesCb(self, msg):
        self.setImageControl('show_sources_enabled', msg.data)

    def setShowGroundCb(self, msg):
        self.setImageControl('show_ground_enabled', msg.data)

    def setShowObstaclesCb(self, msg):
        self.setImageControl('show_obstacles_enabled', msg.data)

    def setImageControl(self, control_name, enabled):
        if control_name not in self.image_controls_dict.keys():
            return
        self.image_controls_dict[control_name] = (enabled == True)
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix + 'image_controls', self.image_controls_dict)
            self.save_config()

    ###############.########################
    # Source Management

    def getAvailableSourceTopics(self):
        status_topics = nepi_sdk.find_topics_by_msg(SOURCE_STATUS_MSG_NAME,
                                                    topics_list = self.active_topics,
                                                    types_list = self.active_topic_types)
        available_source_topics = []
        for status_topic in status_topics:
            source_topic = os.path.dirname(status_topic)
            if os.path.basename(source_topic) in self.OUTPUT_IMG_PRODUCTS:
                continue
            if source_topic not in available_source_topics:
                available_source_topics.append(source_topic)
        return available_source_topics

    def updaterCb(self, timer):
        selected_sources = copy.deepcopy(self.selected_sources)
        active_source_topics = copy.deepcopy(self.active_source_topics)

        ##############
        available_source_topics = self.getAvailableSourceTopics()
        if available_source_topics != self.available_source_topics:
            self.available_source_topics = available_source_topics

        ##############
        # Auto select the first available source when nothing is selected
        if len(selected_sources) == 0 and len(available_source_topics) > 0 \
                and self.auto_select_enabled == True and self.auto_select_active == True:
            self.selected_sources = [available_source_topics[0]]
            selected_sources = copy.deepcopy(self.selected_sources)

        for source_topic in selected_sources:
            if os.path.basename(source_topic) in self.OUTPUT_IMG_PRODUCTS:
                continue
            if source_topic not in active_source_topics and source_topic in available_source_topics:
                self.msg_if.pub_info('Will subscribe to source topic: ' + source_topic, log_name_list = self.log_name_list)
                if source_topic not in self.active_source_topics:
                    self.active_source_topics.append(source_topic)
                self.subscribeSourceTopic(source_topic)

        # Update source subs purge list
        purge_list = []
        for source_topic in active_source_topics:
            if source_topic not in available_source_topics or source_topic not in selected_sources:
                purge_list.append(source_topic)
        for source_topic in purge_list:
            self.msg_if.pub_info('Will unsubscribe from source topic: ' + source_topic, log_name_list = self.log_name_list)
            try:
                self.active_source_topics.remove(source_topic)
            except ValueError:
                pass
            self.unsubscribeSourceTopic(source_topic)

        # Check source connected state
        sources_info_dict = copy.deepcopy(self.sources_info_dict)
        source_connects = []
        for source_topic in sources_info_dict.keys():
            source_connects.append(sources_info_dict[source_topic]['source_connected'])
        source_selected = len(source_connects) > 0
        source_connected = True in source_connects

        # Roll up the reported source characteristics from whichever source is
        # currently being processed, falling back to the first known source.
        self.updateSourceReport(sources_info_dict)

        # Set Obstacles State
        if self.enabled == True:
            if source_selected == False:
                self.msg_str = ProcessStatus.STATE_WAITING
            elif source_connected == False:
                self.msg_str = ProcessStatus.STATE_LISTENING
            else:
                self.msg_str = ProcessStatus.STATE_PROCESSING
        else:
            self.msg_str = ProcessStatus.STATE_LOADED

        nepi_sdk.start_timer_process((1.0), self.updaterCb, oneshot = True)

    def updateSourceReport(self, sources_info_dict):
        report_topic = self.cur_source_topic
        if report_topic not in sources_info_dict.keys():
            topics = list(sources_info_dict.keys())
            report_topic = topics[0] if len(topics) > 0 else None
        if report_topic is None:
            self.has_navpose = False
            self.navpose_topic_connected = False
            self.has_color_image = False
            self.color_image_topic_connected = False
            return

        info_dict = sources_info_dict[report_topic]
        cur_time = nepi_utils.get_time()

        self.min_range_m = info_dict['min_range_m']
        self.max_range_m = info_dict['max_range_m']

        self.navpose_topic = info_dict['navpose_topic']
        self.has_navpose = (self.navpose_topic != '')
        self.navpose_topic_connected = (cur_time - info_dict['navpose_last_connection']) < CONNECTED_TIMEOUT_SEC

        self.color_image_topic = info_dict['color_image_topic']
        self.has_color_image = (self.color_image_topic != '')
        self.color_image_topic_connected = self.has_color_image and info_dict['source_connected']

    def subscribeSourceTopic(self, source_topic):
        if source_topic == "None" or source_topic == "":
            return False

        sources_info_dict = copy.deepcopy(self.sources_info_dict)
        navpose_topic = nepi_sdk.create_namespace(os.path.dirname(source_topic), 'navpose')

        source_subs_dict = {
            'source_sub': {
                    'namespace': source_topic,
                    'msg': Image,
                    'topic': '',
                    'qsize': 1,
                    'callback': self.sourceCb,
                    'callback_args': (source_topic)
            },
            'source_status_sub': {
                    'namespace': source_topic,
                    'msg': DepthMapStatus,
                    'topic': 'status',
                    'qsize': 1,
                    'callback': self.sourceStatusCb,
                    'callback_args': (source_topic)
            },
            'source_navpose_sub': {
                    'namespace': navpose_topic,
                    'msg': NavPose,
                    'topic': '',
                    'qsize': 1,
                    'callback': self.navposeCb,
                    'callback_args': (source_topic)
            }
        }

        if source_topic in sources_info_dict.keys():
            info_dict = sources_info_dict[source_topic]
            if info_dict['source_connecting'] == True or info_dict['source_connected'] == True:
                return False
            self.sources_info_dict[source_topic]['source_connected'] = False
            self.sources_info_dict[source_topic]['source_connecting'] = True
            self.source_ifs_lock.acquire()
            self.source_ifs_dict[source_topic].register_subs(source_subs_dict)
            self.source_ifs_lock.release()
            return True

        self.msg_if.pub_info('Registering to source topic: ' + source_topic, log_name_list = self.log_name_list)

        ####################
        # Create source info dict
        info_dict = dict()
        info_dict['source_topic'] = source_topic
        info_dict['source_connecting'] = True
        info_dict['source_connected'] = False
        info_dict['source_status_dict'] = None

        info_dict['width_px'] = 0
        info_dict['height_px'] = 0
        info_dict['width_deg'] = 110.0
        info_dict['height_deg'] = 70.0
        info_dict['min_range_m'] = 0.0
        info_dict['max_range_m'] = 100.0

        info_dict['color_image_topic'] = ''

        info_dict['navpose_topic'] = navpose_topic
        info_dict['navpose_last_connection'] = 0

        img_namespace = os.path.dirname(source_topic)
        info_dict['img_pub_topic'] = os.path.join(img_namespace, OBSTACLES_IMG_PRODUCT)
        # Where the image pub node will publish this source's two segmentation
        # renders, beside its overlay image.
        info_dict['ground_img_pub_topic'] = os.path.join(img_namespace, GROUND_MAP_IMG_PRODUCT)
        info_dict['obstacles_img_pub_topic'] = os.path.join(img_namespace, OBSTACLES_MAP_IMG_PRODUCT)

        self.sources_info_dict[source_topic] = info_dict

        #####################
        ## Initialize Data Dictionaries
        self.navpose_dict_lock.acquire()
        self.navpose_dict[source_topic] = None
        self.navpose_dict_lock.release()

        ####################
        # Subs Config
        source_subs_if = NodeSubscribersIF(
                subs_dict = source_subs_dict,
                log_name_list = self.log_name_list,
                msg_if = self.msg_if)

        self.source_ifs_lock.acquire()
        self.source_ifs_dict[source_topic] = source_subs_if
        self.source_ifs_lock.release()

        ####################
        # Create source data dict
        source_data_dict = dict()
        source_data_dict['lock'] = threading.Lock()
        source_data_dict['topic'] = source_topic
        source_data_dict['timestamp'] = nepi_utils.get_time()
        source_data_dict['data'] = None
        self.source_dict[source_topic] = source_data_dict

        return True

    def unsubscribeSourceTopic(self, source_topic):
        if source_topic in self.source_ifs_dict.keys():
            self.source_ifs_lock.acquire()
            self.source_ifs_dict[source_topic].unregister_subs()
            self.source_ifs_lock.release()

        if source_topic in self.sources_info_dict.keys():
            self.sources_info_dict[source_topic]['source_connecting'] = False
            self.sources_info_dict[source_topic]['source_connected'] = False
            self.sources_info_dict[source_topic]['source_status_dict'] = None
            self.sources_info_dict[source_topic]['navpose_last_connection'] = 0

        if source_topic in self.source_dict.keys():
            try:
                self.source_dict[source_topic]['lock'].acquire()
                self.source_dict[source_topic]['timestamp'] = nepi_utils.get_time()
                self.source_dict[source_topic]['data'] = None
                self.source_dict[source_topic]['lock'].release()
            except Exception:
                pass

        self.navpose_dict_lock.acquire()
        self.navpose_dict[source_topic] = None
        self.navpose_dict_lock.release()

        return True

    def updateNextTopicCb(self, timer):
        sources_info_dict = copy.deepcopy(self.sources_info_dict)
        active_source_topics = copy.deepcopy(self.active_source_topics)
        if self.enabled == True:
            connected_list = []
            for topic in self.selected_sources:
                if topic in sources_info_dict.keys():
                    if sources_info_dict[topic]['source_connected'] == True:
                        connected_list.append(topic)
            if len(connected_list) == 0:
                self.cur_source_topic = "None"
                self.next_source_topic = "None"
            else:
                cur_source_topic = copy.deepcopy(self.cur_source_topic)

                # Setup next source if needed
                num_connected_list = len(connected_list)
                if cur_source_topic in connected_list:
                    next_ind = connected_list.index(cur_source_topic) + 1
                    if next_ind >= num_connected_list:
                        self.next_source_topic = connected_list[0]
                    else:
                        self.next_source_topic = connected_list[next_ind]
                else:
                    self.next_source_topic = connected_list[0]

                # Check if current source topic is still active
                if cur_source_topic in sources_info_dict.keys():
                    if cur_source_topic not in active_source_topics:
                        self.cur_source_topic = "None"

                # Check if the source topic has been initialized
                if cur_source_topic == "None" and self.next_source_topic != "None":
                    self.got_source_topic = None
                    self.cur_source_topic = copy.deepcopy(self.next_source_topic)
                    self.get_source_topic = copy.deepcopy(self.next_source_topic)

                ##############################
                # Check for non responding source streams
                last_detect_delay = round((nepi_utils.get_time() - self.last_detect_time), 3)
                if self.got_source_topic is None and last_detect_delay > GET_IMAGE_TIMEOUT_SEC:
                    if cur_source_topic in self.sources_info_dict.keys():
                        self.sources_info_dict[cur_source_topic]['source_connected'] = False
                    self.cur_source_topic = self.next_source_topic
                    self.get_source_topic = self.next_source_topic
                elif self.got_source_topic is None:
                    self.cur_source_topic = copy.deepcopy(self.next_source_topic)
                    self.get_source_topic = copy.deepcopy(self.next_source_topic)

        nepi_sdk.start_timer_process((0.01), self.updateNextTopicCb, oneshot = True)

    ###############.########################
    # Data Callbacks

    def sourceStatusCb(self, status_msg, args):
        source_topic = args
        if source_topic not in self.sources_info_dict.keys():
            return
        status_dict = nepi_sdk.convert_msg2dict(status_msg)
        self.sources_info_dict[source_topic]['source_status_dict'] = status_dict
        self.sources_info_dict[source_topic]['width_px'] = status_dict.get('width_px', 0)
        self.sources_info_dict[source_topic]['height_px'] = status_dict.get('height_px', 0)
        self.sources_info_dict[source_topic]['width_deg'] = status_dict.get('width_deg', 110.0)
        self.sources_info_dict[source_topic]['height_deg'] = status_dict.get('height_deg', 70.0)
        self.sources_info_dict[source_topic]['min_range_m'] = status_dict.get('min_range_m', 0.0)
        self.sources_info_dict[source_topic]['max_range_m'] = status_dict.get('max_range_m', 100.0)
        # DepthMapStatus.image_topic names the sibling colour image for this
        # depth map. It is the topic the overlay image pub node renders on.
        self.sources_info_dict[source_topic]['color_image_topic'] = status_dict.get('image_topic', '')
        navpose_topic = status_dict.get('navpose_topic', '')
        if navpose_topic != '':
            self.sources_info_dict[source_topic]['navpose_topic'] = navpose_topic

    def sourceCb(self, msg, args):
        source_topic = args
        if source_topic not in self.sources_info_dict.keys():
            return

        self.sources_info_dict[source_topic]['source_connected'] = True
        self.sources_info_dict[source_topic]['source_connecting'] = False

        timestamp = copy.deepcopy(float(msg.header.stamp.to_sec()))

        ###############################
        source_receive_latency = round(nepi_utils.get_time() - timestamp, 3)
        source_receive_delay = (nepi_utils.get_time() - self.last_receive_source_time)
        if source_receive_delay > 0.01:
            self.source_receive_latencies.pop(0)
            self.source_receive_latencies.append(source_receive_latency)

            self.source_receive_rates.pop(0)
            self.source_receive_rates.append(round(1.0 / source_receive_delay, 3))

            self.last_receive_source_time = nepi_utils.get_time()

        #####################################
        if source_topic == self.get_source_topic:
            self.got_source_topic = source_topic
            self.last_detect_time = nepi_utils.get_time()

            np_depth_map = nepi_img.rosimg_to_cv2img(msg)

            if source_topic in self.source_dict.keys():
                try:
                    self.source_dict[source_topic]['lock'].acquire()
                    self.source_dict[source_topic]['topic'] = source_topic
                    self.source_dict[source_topic]['timestamp'] = timestamp
                    self.source_dict[source_topic]['data'] = np_depth_map
                    self.source_dict[source_topic]['lock'].release()
                except Exception as e:
                    self.msg_if.pub_warn("Failed to write source dict " + str(source_topic) + " : " + str(e), log_name_list = self.log_name_list, throttle_s = 5.0)

    def navposeCb(self, msg, args):
        source_topic = args
        navpose_dict = nepi_nav.convert_navpose_msg2dict(msg)

        self.navpose_dict_lock.acquire()
        self.navpose_dict[source_topic] = navpose_dict
        self.navpose_dict_lock.release()

        if source_topic in self.sources_info_dict.keys():
            self.sources_info_dict[source_topic]['navpose_last_connection'] = nepi_utils.get_time()

    ###############.########################
    # Process Loop

    def processObstaclesCb(self, timer):
        start_time = nepi_utils.get_time()
        if self.is_processing == True:
            nepi_sdk.start_timer_process((0.01), self.processObstaclesCb, oneshot = True)
            return

        ##############################
        ### Get depth map
        ###############################
        np_depth_map = None
        source_timestamp = None
        source_topic = copy.deepcopy(self.got_source_topic)
        if source_topic is not None and source_topic in self.source_dict.keys():
            self.source_dict[source_topic]['lock'].acquire()
            source_timestamp = self.source_dict[source_topic]['timestamp']
            np_depth_map = copy.deepcopy(self.source_dict[source_topic]['data'])
            self.source_dict[source_topic]['data'] = None
            self.source_dict[source_topic]['lock'].release()
            if np_depth_map is not None:
                self.got_source_topic = None

        #####################################
        if np_depth_map is not None and self.enabled == True and source_topic in self.sources_info_dict.keys():
            self.is_processing = True
            try:
                preprocess_time = round((nepi_utils.get_time() - start_time), 3)
                self.preprocess_times.pop(0)
                self.preprocess_times.append(preprocess_time)

                self.preprocess_latencies.pop(0)
                self.preprocess_latencies.append(nepi_utils.get_time() - source_timestamp)

                self.preprocess_rates.pop(0)
                self.preprocess_rates.append(round(1.0 / max((nepi_utils.get_time() - start_time), 0.001), 3))

                ##############################
                # Gather the process inputs
                status_dict = copy.deepcopy(self.sources_info_dict[source_topic]['source_status_dict'])
                if status_dict is None:
                    status_dict = nepi_sdk.convert_msg2dict(DepthMapStatus())

                self.navpose_dict_lock.acquire()
                navpose_dict = copy.deepcopy(self.navpose_dict.get(source_topic, None))
                self.navpose_dict_lock.release()
                if navpose_dict is None:
                    navpose_dict = nepi_nav.convert_navpose_msg2dict(NavPose())

                controls_dict = self.get_controls_dict()


                ##############################
                # Process Obstacles
                obstacles_dict_list = []
                depth_map_ground = None
                depth_map_obstacles = None
                start_process_time = nepi_utils.get_time()
                obstacles_timestamp = nepi_utils.get_time()
                try:
                    [obstacles_dict_list, depth_map_ground, depth_map_obstacles, self.data_dict] = self.processResults(np_depth_map,
                                                                                                    status_dict,
                                                                                                    navpose_dict,
                                                                                                    self.data_dict,
                                                                                                    controls_dict)
                    self.first_process_complete = True
                except Exception as e:
                    self.msg_if.pub_warn("Failed to process obstacles with exception: " + str(e), log_name_list = self.log_name_list, throttle_s = 5.0)
                    obstacles_dict_list = []

                ###############################
                process_time = round((nepi_utils.get_time() - start_process_time), 3)
                self.process_times.pop(0)
                self.process_times.append(process_time)

                ##################################
                self.publishObstaclesData(source_topic,
                                          source_timestamp,
                                          obstacles_dict_list,
                                          obstacles_timestamp,
                                          navpose_dict,
                                          depth_map_ground,
                                          depth_map_obstacles)
                ##################################

                self.process_latencies.pop(0)
                self.process_latencies.append(nepi_utils.get_time() - source_timestamp)

                process_delay = nepi_utils.get_time() - self.last_process_time
                self.process_rates.pop(0)
                self.process_rates.append(round(1.0 / max(process_delay, 0.001), 3))
                self.last_process_time = nepi_utils.get_time()
            finally:
                self.is_processing = False

        #####################################
        cycle_time = nepi_utils.get_time() - start_time
        max_rate = self.max_process_rate_hz
        if max_rate < MIN_MAX_RATE:
            max_rate = MIN_MAX_RATE
        delay_time = (float(1) / max_rate) - cycle_time
        if delay_time < 0.01:
            delay_time = 0.01

        nepi_sdk.start_timer_process((delay_time), self.processObstaclesCb, oneshot = True)

    def publishObstaclesData(self, source_topic,
                                    source_timestamp,
                                    obstacles_dict_list,
                                    obstacles_timestamp,
                                    navpose_dict,
                                    depth_map_ground = None,
                                    depth_map_obstacles = None):
        obstacle_count = len(obstacles_dict_list)
        if source_topic not in copy.deepcopy(self.active_source_topics):
            return

        obstacle_msg_list = []
        for obstacle_dict in obstacles_dict_list:
            obstacle_msg = self.getObstacleMsg(obstacle_dict)
            if obstacle_msg is not None:
                obstacle_msg_list.append(obstacle_msg)

        obstacles_msg = Obstacles()
        obstacles_msg.timestamp = float(obstacles_timestamp)

        obstacles_msg.process_name = self.node_name
        obstacles_msg.process_namespace = self.obstacles_namespace

        obstacles_msg.source_topic = source_topic
        obstacles_msg.source_timestamp = float(source_timestamp)

        obstacles_msg.navpose_frame = str(navpose_dict.get('navpose_frame', '')) if navpose_dict is not None else ''
        navpose_msg = None
        try:
            navpose_msg = nepi_nav.convert_navpose_dict2msg(navpose_dict)
        except Exception:
            navpose_msg = None
        obstacles_msg.navpose_msg = navpose_msg if navpose_msg is not None else NavPose()

        obstacles_msg.obstacles = obstacle_msg_list

        # The segmentation maps travel on their own topic, built from the same
        # cycle's data and published immediately after the obstacle list so a
        # consumer pairing on source_topic + source_timestamp sees the two
        # arrive together.
        depth_map_msg = ObstaclesDepthMap()
        depth_map_msg.timestamp = obstacles_msg.timestamp

        depth_map_msg.process_name = obstacles_msg.process_name
        depth_map_msg.process_namespace = obstacles_msg.process_namespace

        depth_map_msg.source_topic = obstacles_msg.source_topic
        depth_map_msg.source_timestamp = obstacles_msg.source_timestamp

        depth_map_msg.navpose_frame = obstacles_msg.navpose_frame
        depth_map_msg.navpose_msg = obstacles_msg.navpose_msg

        depth_map_msg.depth_map_ground = self.getDepthMapImgMsg(depth_map_ground)
        depth_map_msg.depth_map_obstacles = self.getDepthMapImgMsg(depth_map_obstacles)

        if self.node_if is not None:
            self.node_if.publish_pub('obstacles_pub', obstacles_msg)
            self.node_if.publish_pub('obstacles_all_pub', obstacles_msg)
            self.node_if.publish_pub('obstacles_depth_map_pub', depth_map_msg)

        self.saveObstaclesData(obstacles_msg, obstacles_timestamp)

        if obstacle_count > 0:
            if 'obstacles_trigger' in self.triggers_dict.keys():
                trigger_dict = self.triggers_dict['obstacles_trigger']
                trigger_dict['time'] = nepi_utils.get_time()
                try:
                    self.triggers_if.publish_trigger(trigger_dict)
                except Exception:
                    pass
            self.processing_state = True
            self.process_state = True

    def saveObstaclesData(self, obstacles_msg, timestamp):
        # Mirrors DetectionsIF.publish_data: gate on the rate/snapshot check
        # first, then convert to a dict, because SaveDataIF writes dicts as YAML
        # and cannot infer a type from a ROS message. The message no longer
        # carries the two depth maps, so nothing has to be stripped here -- the
        # saved visual form of that data is the obstacles_image data product.
        if self.save_data_if is None or obstacles_msg is None:
            return
        should_save = self.save_data_if.data_product_should_save('obstacles') == True
        snapshot_enabled = self.save_data_if.data_product_snapshot_enabled('obstacles') == True
        if should_save == False and snapshot_enabled == False:
            return
        try:
            data_dict = nepi_sdk.convert_msg2dict(obstacles_msg)
            self.save_data_if.save('obstacles', data_dict, timestamp = timestamp)
        except Exception as e:
            self.msg_if.pub_warn("Failed to save obstacles data: " + str(e), log_name_list = self.log_name_list, throttle_s = 5.0)

    def getObstacleMsg(self, obstacle_dict):
        obstacle_msg = None
        try:
            obstacle_msg = Obstacle()
            obstacle_msg.timestamp = float(obstacle_dict['timestamp'])
            obstacle_msg.name = obstacle_dict['name']
            obstacle_msg.id = int(obstacle_dict['id'])
            obstacle_msg.uid = obstacle_dict['uid']
            obstacle_msg.confidence = float(obstacle_dict['confidence'])

            obstacle_msg.xmin_pixel = int(obstacle_dict['xmin_pixel'])
            obstacle_msg.ymin_pixel = int(obstacle_dict['ymin_pixel'])
            obstacle_msg.xmax_pixel = int(obstacle_dict['xmax_pixel'])
            obstacle_msg.ymax_pixel = int(obstacle_dict['ymax_pixel'])
            obstacle_msg.width_pixels = int(obstacle_dict['width_pixels'])
            obstacle_msg.height_pixels = int(obstacle_dict['height_pixels'])
            obstacle_msg.area_pixels = float(obstacle_dict['area_pixels'])
            obstacle_msg.area_ratio = float(obstacle_dict['area_ratio'])
            obstacle_msg.vel_pixels.x = float(obstacle_dict['vel_pixels'][0])
            obstacle_msg.vel_pixels.y = float(obstacle_dict['vel_pixels'][1])
            obstacle_msg.vel_pixels.z = float(obstacle_dict['vel_pixels'][2])

            # Position Data ENU Reference Frame
            obstacle_msg.range_m = float(obstacle_dict['range_m'])
            obstacle_msg.azimuth_deg = float(obstacle_dict['azimuth_deg'])
            obstacle_msg.elevation_deg = float(obstacle_dict['elevation_deg'])
        except Exception as e:
            obstacle_msg = None
            self.msg_if.pub_warn("Failed to convert obstacle dict: " + str(e), log_name_list = self.log_name_list, throttle_s = 5.0)
        return obstacle_msg

    def getDepthMapImgMsg(self, np_depth_map):
        # A 0x0 Image is the wire representation of "not produced this cycle".
        if np_depth_map is None:
            return Image()
        img_msg = None
        try:
            img_msg = nepi_img.cv2img_to_rosimg(np_depth_map, encoding = '32FC1')
        except Exception as e:
            img_msg = None
            self.msg_if.pub_warn("Failed to convert depth map to img msg: " + str(e), log_name_list = self.log_name_list, throttle_s = 5.0)
        return img_msg if img_msg is not None else Image()

    ###############.########################
    # Status

    def getProcessStatus(self):
        self.process_status_msg.name = self.process_name
        self.process_status_msg.group = 'PROCESS'
        self.process_status_msg.description = self.description

        self.process_status_msg.node_name = self.node_name
        self.process_status_msg.namespace = self.obstacles_namespace

        self.process_status_msg.data_products = self.data_products
        self.process_status_msg.save_data_topic = self.save_data_namespace

        self.process_status_msg.max_process_rate_hz = self.max_process_rate_hz

        self.process_status_msg.multi_source_enabled = True
        self.process_status_msg.available_source_topics = self.available_source_topics
        self.process_status_msg.auto_select_enabled = self.auto_select_enabled
        if self.auto_select_enabled == False:
            self.auto_select_active = False
        self.process_status_msg.auto_select_active = self.auto_select_active
        self.process_status_msg.selected_sources = self.selected_sources

        sources_info_dict = copy.deepcopy(self.sources_info_dict)
        active_source_topics = copy.deepcopy(self.active_source_topics)

        source_connects = []
        source_pub_namespaces = []
        for source_topic in sources_info_dict.keys():
            source_connects.append(sources_info_dict[source_topic]['source_connected'])
            source_pub_namespaces.append(self.obstacles_namespace)
        self.process_status_msg.sources_connected = source_connects
        self.process_status_msg.sources_pub_namespaces = source_pub_namespaces

        source_selected = len(source_connects) > 0
        self.process_status_msg.source_selected = source_selected
        source_connected = True in source_connects
        self.process_status_msg.source_connected = source_connected

        self.process_status_msg.has_image_pub = True
        self.process_status_msg.image_pub_name = 'obstacles_image'
        self.process_status_msg.image_pub_enabled = self.imaging_enabled
        self.process_status_msg.max_image_pub_rate_hz = self.max_image_pub_rate_hz
        self.process_status_msg.use_last_image = self.use_last_image

        imaging_source_topics = []
        imaging_pub_topics = []
        for source_topic in sources_info_dict.keys():
            if source_topic in active_source_topics:
                imaging_source_topics.append(source_topic)
                imaging_pub_topics.append(sources_info_dict[source_topic]['img_pub_topic'])
        self.process_status_msg.imaging_source_topics = imaging_source_topics
        self.process_status_msg.imaging_pub_topics = imaging_pub_topics
        self.imaging_source_topics = imaging_source_topics

        #################
        self.process_status_msg.enabled = self.enabled
        self.process_status_msg.running = self.enabled and source_selected and source_connected
        self.process_status_msg.state = self.process_state
        self.process_status_msg.msg_str = self.msg_str

        #################
        source_receive_latency = sum(self.source_receive_latencies) / len(self.source_receive_latencies)
        self.process_status_msg.avg_source_latency = source_receive_latency
        self.process_status_msg.avg_source_rate = sum(self.source_receive_rates) / len(self.source_receive_rates)

        self.process_status_msg.avg_preprocess_latency = sum(self.preprocess_latencies) / len(self.preprocess_latencies)
        self.process_status_msg.avg_preprocess_rate = sum(self.preprocess_rates) / len(self.preprocess_rates)

        self.process_status_msg.avg_process_latency = sum(self.process_latencies) / len(self.process_latencies)
        self.process_status_msg.avg_process_rate = sum(self.process_rates) / len(self.process_rates)

        avg_process_time = sum(self.process_times) / len(self.process_times)
        if avg_process_time > 0.001:
            max_process_rate = 1.0 / avg_process_time
        else:
            max_process_rate = 0
        self.process_status_msg.max_process_rate = max_process_rate

        #################
        self.process_status_msg.show_selector = True
        self.process_status_msg.show_controls = True
        self.process_status_msg.show_data = True

        return self.process_status_msg

    def publishStatusCb(self, timer):
        self.publish_status()
        self.process_state = False
