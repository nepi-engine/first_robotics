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
import math
import threading

import numpy as np

from std_msgs.msg import Bool, Empty, Float32
from sensor_msgs.msg import Image

from nepi_interfaces.msg import DepthMapStatus
from nepi_interfaces.msg import Detections
from nepi_interfaces.msg import ImageMouseEvent
from nepi_interfaces.msg import ImagePixel
from nepi_interfaces.msg import MgrSystemStatus
from nepi_interfaces.msg import Targets

from nepi_app_auto_move.msg import NepiAppAutoMoveStatus

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_system
from nepi_sdk import nepi_img

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeSubscribersIF, NodeClassIF
from nepi_api.system_if import ControlsIF

from nepi_api.connect_device_if_rbx import ConnectRBXDeviceIF
from nepi_api.connect_data_if import ConnectImageIF


# The obstacle overlay layer is optional. nepi_app_obstacles ships its own
# Obstacles message, and a device that does not carry the obstacles app has no
# such module to import. Guarded rather than declared as a package dependency so
# this app builds and runs either way: without it the obstacles layer reports
# itself not found and draws nothing, and every other layer is unaffected.
try:
    from nepi_app_obstacles.msg import Obstacles
    HAS_OBSTACLES_MSG = True
except ImportError:
    Obstacles = None
    HAS_OBSTACLES_MSG = False


SYSTEM_ALL_TOPIC = 'all'

#########################################
# AutoMove IF
#########################################

# Connect Names
#
# One connect name per connect IF. Each names the <app>/<connect_name> connect
# namespace that IF owns, and is the exact string the matching Nepi_IF_Connect*
# component in NepiAppAutoMove.js binds to. Both are declared rather than
# defaulted because the class defaults do not survive contact with a second
# instance -- every ConnectImageIF would land on ConnectDataIF's 'data_connect'
# -- so each is passed explicitly to keep the RUI binding unambiguous and
# greppable from both sides.
RBX_CONNECT_NAME = 'rbx_connect'
IMAGE_CONNECT_NAME = 'image_connect'

# Controls name of the app's own control set: the <app>/controls namespace one
# ControlsIF owns and the Nepi_IF_Controls box in this app's RUI column binds to.
CONTROLS_NAME = 'controls'

# Sibling data product names. A NEPI image source publishes its data products
# flat under one parent namespace, so the depth map for <P>/color_image is
# <P>/depth_map. The colourized render of a depth map is nested one level
# further, at <depth map>/depth_map_image -- the platform convention DepthMapIF
# sets, where raw range data is <name> and its viewable render is <name>_image.
#
# DepthMapStatus.image_topic is deliberately NOT used to find the depth map
# image: that field names the sibling COLOUR image, not the depth map's render.
DEPTH_MAP_TOPIC = 'depth_map'
DEPTH_MAP_IMAGE_TOPIC = 'depth_map_image'

# Collective fan-out topics. An AI detector publishes detections and targets
# under its own node namespace and republishes them here, and an obstacles
# process does the same with its obstacle list. Subscribing here once costs one
# subscriber instead of one per producer, and each message names the source it
# was computed from, which is the only way to tie it back to a selected image.
DETECTIONS_ALL_TOPIC = 'detections'
TARGETS_ALL_TOPIC = 'targets'
OBSTACLES_ALL_TOPIC = 'obstacles'

# Topic the RUI image viewer publishes its ImageMouseEvent clicks on. Passed to
# Nepi_IF_ImageViewer as its mouse_event_topic prop, which overrides the
# viewer's default of <image topic>/mouse_event -- the app wants the clicks, not
# the image source.
MOUSE_EVENT_TOPIC = 'image_mouse_event'

# Image data product the image pub node publishes, one per active source.
OVERLAY_IMG_PRODUCT = 'auto_move_image'

# Node file for the overlay image publisher this IF launches. It installs into
# the app package's lib folder via catkin_install_python, which sits beside the
# nepi_api lib folder reported by nepi_system.get_system_folders().
IMG_PUB_PKG_NAME = 'nepi_app_auto_move'
IMG_PUB_NODE_FILE = 'auto_move_app_img_pub_node.py'

# Overlay transparency, 0.0 fully opaque through 1.0 invisible. The image pub
# node blends at (1.0 - transparency).
DEFAULT_DEPTH_MAP_TRANSPARENCY = 0.5

# Largest forward distance one click may request, and the bounds the setter
# clamps an operator entry into.
DEFAULT_MAX_MOVE_M = 5.0
MIN_MAX_MOVE_M = 0.1
MAX_MAX_MOVE_M = 100.0

# Fallback field of view when neither the depth map status nor the image status
# reports one. Same numbers nepi_obstacles falls back to.
DEFAULT_WIDTH_DEG = 110.0
DEFAULT_HEIGHT_DEG = 70.0

# Half-width of the square pixel window a click samples for its depth. One pixel
# is one return and a single bad return would send the robot somewhere it was
# never asked to go; the median over a small patch survives speckle without
# smearing across a real depth edge.
DEPTH_SAMPLE_HALF_PX = 2

# How long after the last matching message a companion still counts as found.
CONNECTED_TIMEOUT_SEC = 2

# How long a goto step may run before the process gives up on it. The device
# enforces its own command timeout as well; this is the outer bound so a device
# that stops reporting cannot strand the process in MOVING forever.
GOTO_STEP_TIMEOUT_SEC = 60

# How long to wait for the device to report busy after a step is issued, before
# treating the step as complete. A short move can finish inside one status
# period, which would otherwise read as "never started".
GOTO_BUSY_WAIT_SEC = 3


class AutoMoveIF:

    namespace = '~'
    all_namespace = None

    status_msg = NepiAppAutoMoveStatus()

    node_if = None
    controls_if = None
    rbx_if = None
    image_if = None

    node_if_prefix = 'auto_move_'

    # Selected sources
    image_topic = 'None'
    image_connected = False
    image_status_dict = None
    image_width_px = 0
    image_height_px = 0

    rbx_namespace = 'None'
    rbx_connected = False
    rbx_ready = False

    # Resolved companion topics. Each is what the app looked for; the matching
    # found flag says whether it is live.
    depth_map_topic = ''
    depth_map_found = False
    depth_map_image_topic = ''
    depth_map_image_found = False

    objects_topic = ''
    objects_found = False
    targets_topic = ''
    targets_found = False
    obstacles_topic = ''
    obstacles_found = False

    overlay_image_topic = ''

    # Newest depth map frame, as the raw numpy array the source published. Held
    # under its own lock because the click path reads it on a subscriber thread
    # while the depth map subscriber replaces it on another.
    depth_map_slot = None
    depth_map_stamp = 0
    depth_map_lock = threading.Lock()

    depth_map_status_dict = None
    depth_map_last_time = 0
    depth_map_image_last_time = 0

    # Newest matching detection / target / obstacle lists for the selected
    # source, as plain dicts. Replaced whole, never mutated, so a plain read is
    # a consistent read.
    objects_list = []
    targets_list = []
    obstacles_list = []
    objects_last_time = 0
    targets_last_time = 0
    obstacles_last_time = 0

    # Dynamic subscriber set for the selected image's companions, rebuilt
    # whenever the selection changes.
    source_subs_if = None
    subscribed_image_topic = 'None'

    # Click and goto state
    has_click = False
    click_pixel_x = 0
    click_pixel_y = 0
    click_x_ratio = 0.0
    click_y_ratio = 0.0
    click_depth_valid = False
    click_msg = 'No click yet'

    goto_x_m = 0.0
    goto_y_m = 0.0
    goto_z_m = 0.0
    goto_clamped = False

    goto_state = NepiAppAutoMoveStatus.GOTO_STATE_IDLE
    goto_msg = 'Idle'
    goto_step = 0
    goto_step_count = 0
    goto_plan = []
    goto_cancel_requested = False
    goto_step_start_time = 0
    goto_step_issued = False
    goto_saw_busy = False

    # One param dict for every overlay view control. initCb copies key by key, so
    # adding a key here is safe against a saved config written before it existed.
    image_controls_dict = dict(
        depth_map_transparency = DEFAULT_DEPTH_MAP_TRANSPARENCY,
        show_depth_map_enabled = True,
        show_objects_enabled = True,
        show_targets_enabled = True,
        show_obstacles_enabled = True,
        show_crosshair_enabled = True
    )

    max_move_m = DEFAULT_MAX_MOVE_M

    active_nodes = []
    active_topics = []
    active_topic_types = []
    active_services = []

    api_lib_folder = '/opt/nepi/nepi_engine/lib/nepi_api'
    img_pub_lib_folder = '/opt/nepi/nepi_engine/lib/nepi_app_auto_move'
    launch_node_process = None
    pub_img_node_name = ""
    pub_img_namespace = ""

    save_config_enabled = True
    ready = False

    def __init__(self,
                namespace,
                description,
                controls_dict,
                planMoveFunction,
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
        self.msg_if.pub_debug("Starting AutoMove IF Initialization Processes", log_name_list = self.log_name_list)

        ##############################
        # Init Class Variables

        if namespace is None:
            namespace = self.node_namespace
        self.namespace = nepi_sdk.get_full_namespace(namespace)

        self.all_namespace = nepi_sdk.create_namespace(self.base_namespace, SYSTEM_ALL_TOPIC)

        self.enable_image_pub = enable_image_pub
        self.description = description
        self.planMove = planMoveFunction

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
        # NOTE: for params the registry key IS the wire name (namespace + key),
        # so the auto_move_ prefix is deliberate and load-bearing, not just
        # collision insurance.
        self.PARAMS_DICT = {
            self.node_if_prefix + 'max_move_m': {
                'namespace': self.namespace,
                'factory_val': DEFAULT_MAX_MOVE_M
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
            self.node_if_prefix + 'status_pub': {
                'msg': NepiAppAutoMoveStatus,
                'namespace': self.namespace,
                'topic': 'status',
                'qsize': 1,
                'latch': True
            },
        }

        # Subs Config Dict ####################
        self.SUBS_DICT = {
            ############
            # Click to goto
            ############
            self.node_if_prefix + 'image_mouse_event': {
                'namespace': self.namespace,
                'topic': MOUSE_EVENT_TOPIC,
                'msg': ImageMouseEvent,
                'qsize': 10,
                'callback': self.mouseEventCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'set_goto_x': {
                'namespace': self.namespace,
                'topic': 'set_goto_x',
                'msg': Float32,
                'qsize': 10,
                'callback': self.setGotoXCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'set_goto_y': {
                'namespace': self.namespace,
                'topic': 'set_goto_y',
                'msg': Float32,
                'qsize': 10,
                'callback': self.setGotoYCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'set_goto_z': {
                'namespace': self.namespace,
                'topic': 'set_goto_z',
                'msg': Float32,
                'qsize': 10,
                'callback': self.setGotoZCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'set_max_move': {
                'namespace': self.namespace,
                'topic': 'set_max_move',
                'msg': Float32,
                'qsize': 10,
                'callback': self.setMaxMoveCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'goto_trigger': {
                'namespace': self.namespace,
                'topic': 'goto_trigger',
                'msg': Empty,
                'qsize': 10,
                'callback': self.gotoTriggerCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'goto_cancel': {
                'namespace': self.namespace,
                'topic': 'goto_cancel',
                'msg': Empty,
                'qsize': 10,
                'callback': self.gotoCancelCb,
                'callback_args': ()
            },
            ############
            # Image Overlay Controls
            ############
            self.node_if_prefix + 'set_depth_map_transparency': {
                'namespace': self.namespace,
                'topic': 'set_depth_map_transparency',
                'msg': Float32,
                'qsize': 10,
                'callback': self.setDepthMapTransparencyCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'set_show_depth_map': {
                'namespace': self.namespace,
                'topic': 'set_show_depth_map',
                'msg': Bool,
                'qsize': 10,
                'callback': self.setShowDepthMapCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'set_show_objects': {
                'namespace': self.namespace,
                'topic': 'set_show_objects',
                'msg': Bool,
                'qsize': 10,
                'callback': self.setShowObjectsCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'set_show_targets': {
                'namespace': self.namespace,
                'topic': 'set_show_targets',
                'msg': Bool,
                'qsize': 10,
                'callback': self.setShowTargetsCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'set_show_obstacles': {
                'namespace': self.namespace,
                'topic': 'set_show_obstacles',
                'msg': Bool,
                'qsize': 10,
                'callback': self.setShowObstaclesCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'set_show_crosshair': {
                'namespace': self.namespace,
                'topic': 'set_show_crosshair',
                'msg': Bool,
                'qsize': 10,
                'callback': self.setShowCrosshairCb,
                'callback_args': ()
            },
            ############
            # Collective data fan-out
            ############
            self.node_if_prefix + 'all_detections': {
                'namespace': self.all_namespace,
                'topic': DETECTIONS_ALL_TOPIC,
                'msg': Detections,
                'qsize': 1,
                'callback': self.detectionsCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'all_targets': {
                'namespace': self.all_namespace,
                'topic': TARGETS_ALL_TOPIC,
                'msg': Targets,
                'qsize': 1,
                'callback': self.targetsCb,
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

        # The obstacles fan-out subscriber exists only where the obstacles
        # message package does. Added to the dict rather than registered
        # separately so it shares the same registry and the same domain prefix.
        if HAS_OBSTACLES_MSG == True:
            self.SUBS_DICT[self.node_if_prefix + 'all_obstacles'] = {
                'namespace': self.all_namespace,
                'topic': OBSTACLES_ALL_TOPIC,
                'msg': Obstacles,
                'qsize': 1,
                'callback': self.obstaclesCb,
                'callback_args': ()
            }
        else:
            self.msg_if.pub_info("Obstacles message package not installed, obstacle overlay disabled",
                                 log_name_list = self.log_name_list)

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
                        controls_name = CONTROLS_NAME,
                        controls_display_name = 'Auto Move Controls',
                        controls_description = self.description,
                        controls_init_dict = self.controls_dict,
                        show_controls = True,
                        has_show_control = False,
                        log_name = CONTROLS_NAME,
                        log_name_list = self.log_name_list,
                        msg_if = self.msg_if)
        self.controls_if.wait_for_controls_ready()

        ###############################
        # Create Connect IFs
        self.setupConnectIFs()

        ###############################
        # Launch the overlay image publisher node
        self.launch_image_pub_node()

        ##########################
        # Complete Initialization

        # Start Timer Processes
        nepi_sdk.start_timer_process((1.0), self.publishStatusCb)
        nepi_sdk.start_timer_process((1.0), self.updaterCb, oneshot = True)
        nepi_sdk.start_timer_process((0.2), self.gotoProcessCb, oneshot = True)

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
        """Return the fully-resolved namespace this app owns.

        Returns:
            str: The app node namespace carrying the status topic and every
                command topic.
        """
        return self.namespace

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

    def get_controls_namespace(self):
        """Return the fully-qualified namespace of this app's control set.

        Built from the app namespace rather than returned straight from
        ControlsIF.get_namespace() so the value is guaranteed to carry a leading
        slash. The RUI appends '/status' to it and hands the result to
        rosbridge, where a name with no leading slash resolves at the global
        root instead of under the device namespace.

        Returns:
            str: The ``<app>/controls`` namespace, fully qualified.
        """
        return nepi_sdk.get_full_namespace(nepi_sdk.create_namespace(self.namespace, CONTROLS_NAME))

    def get_connect_namespace(self, connect_name):
        """Return the fully-qualified namespace of one of this app's connect IFs.

        Args:
            connect_name (str): The connect name the IF was constructed with,
                e.g. 'rbx_connect' or 'image_connect'.

        Returns:
            str: The ``<app>/<connect_name>`` namespace, fully qualified. Same
                leading-slash requirement as get_controls_namespace().
        """
        return nepi_sdk.get_full_namespace(nepi_sdk.create_namespace(self.namespace, connect_name))

    def launch_image_pub_node(self):
        """Launch the auto move overlay image publisher node as a subprocess.

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
            self.msg_if.pub_info("Launching Auto Move Img Node with settings " + str([IMG_PUB_PKG_NAME, IMG_PUB_NODE_FILE, node_name]), log_name_list = self.log_name_list)
            [success, msg, sub_process] = nepi_sdk.launch_node(IMG_PUB_PKG_NAME, IMG_PUB_NODE_FILE, node_name, namespace = launch_namespace)
            if success == True:
                self.launch_node_process = sub_process
                self.pub_img_node_name = node_name
                self.pub_img_namespace = node_namespace
            self.msg_if.pub_info("Node launch return msg: " + str(msg), log_name_list = self.log_name_list)

    def kill_image_pub_node(self):
        """Terminate the running auto move overlay image publisher node.

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

    def save_config(self):
        """Persist this interface's parameters through the config manager."""
        if self.save_config_enabled == True:
            if self.node_if is not None:
                self.node_if.save_config()

    def publish_status(self):
        """Assemble and publish the auto move status message.

        Populates every field from current internal state and publishes on
        ``<app>/status``.
        """
        status_msg = NepiAppAutoMoveStatus()

        status_msg.rbx_connect_namespace = self.get_connect_namespace(RBX_CONNECT_NAME)
        status_msg.rbx_namespace = self.rbx_namespace
        status_msg.rbx_connected = self.rbx_connected
        status_msg.rbx_ready = self.rbx_ready

        status_msg.image_connect_namespace = self.get_connect_namespace(IMAGE_CONNECT_NAME)
        status_msg.image_topic = self.image_topic
        status_msg.image_connected = self.image_connected
        status_msg.image_width_px = int(self.image_width_px)
        status_msg.image_height_px = int(self.image_height_px)

        status_msg.depth_map_topic = self.depth_map_topic
        status_msg.depth_map_found = self.depth_map_found
        status_msg.depth_map_image_topic = self.depth_map_image_topic
        status_msg.depth_map_image_found = self.depth_map_image_found

        status_msg.objects_topic = self.objects_topic
        status_msg.objects_found = self.objects_found
        status_msg.targets_topic = self.targets_topic
        status_msg.targets_found = self.targets_found
        status_msg.obstacles_topic = self.obstacles_topic
        status_msg.obstacles_found = self.obstacles_found

        status_msg.mouse_event_topic = nepi_sdk.create_namespace(self.namespace, MOUSE_EVENT_TOPIC)
        status_msg.overlay_image_topic = self.overlay_image_topic

        status_msg.depth_map_transparency = float(self.image_controls_dict['depth_map_transparency'])
        status_msg.show_depth_map_enabled = self.image_controls_dict['show_depth_map_enabled']
        status_msg.show_objects_enabled = self.image_controls_dict['show_objects_enabled']
        status_msg.show_targets_enabled = self.image_controls_dict['show_targets_enabled']
        status_msg.show_obstacles_enabled = self.image_controls_dict['show_obstacles_enabled']
        status_msg.show_crosshair_enabled = self.image_controls_dict['show_crosshair_enabled']

        click_pixel = ImagePixel()
        click_pixel.x = int(self.click_pixel_x)
        click_pixel.y = int(self.click_pixel_y)
        status_msg.last_click_pixel = click_pixel
        status_msg.last_click_x_ratio = float(self.click_x_ratio)
        status_msg.last_click_y_ratio = float(self.click_y_ratio)
        status_msg.has_click = self.has_click
        status_msg.click_depth_valid = self.click_depth_valid
        status_msg.click_msg = self.click_msg

        status_msg.goto_x_m = float(self.goto_x_m)
        status_msg.goto_y_m = float(self.goto_y_m)
        status_msg.goto_z_m = float(self.goto_z_m)
        status_msg.max_move_m = float(self.max_move_m)
        status_msg.goto_clamped = self.goto_clamped

        status_msg.goto_state = self.goto_state
        status_msg.goto_msg = self.goto_msg
        status_msg.goto_step = int(self.goto_step)
        status_msg.goto_step_count = int(self.goto_step_count)

        status_msg.controls_namespace = self.get_controls_namespace()
        status_msg.controls_ready = False if self.controls_if is None else self.controls_if.get_controls_ready_state()

        self.status_msg = status_msg

        if self.node_if is not None:
            self.node_if.publish_pub(self.node_if_prefix + 'status_pub', status_msg)

    def unregister(self):
        """Tear down the image pub node and this interface's ROS registrations."""
        self.kill_image_pub_node()
        self.unsubscribeSourceTopics()
        if self.rbx_if is not None:
            self.rbx_if.unregister()
        if self.image_if is not None:
            self.image_if.unregister()
        if self.node_if is not None:
            self.node_if.unregister_class()

    ###############################
    # Class Private Methods
    ###############################

    def setupConnectIFs(self):
        # Two connect IFs, each owning its own selector in this app's RUI:
        # the robot to drive and the image to click on. Both are built with
        # show_selector=True and the controls and data panels hidden -- this app
        # renders its own overlay viewer and its own goto controls, so the
        # connect IFs are used purely as source selectors.
        #
        # Neither is given a dataCB. The RBX device is polled for status in
        # updaterCb, and the image frames are consumed by the image pub node,
        # not here -- registering an image dataCB would decode every frame on
        # this node for nothing.
        self.rbx_if = ConnectRBXDeviceIF(
                        connect_name = RBX_CONNECT_NAME,
                        show_selector = True,
                        show_controls = False,
                        show_data = False,
                        log_name = RBX_CONNECT_NAME,
                        msg_if = self.msg_if)

        self.image_if = ConnectImageIF(
                        connect_name = IMAGE_CONNECT_NAME,
                        show_selector = True,
                        show_controls = False,
                        show_data = False,
                        statusCb = self.imageStatusCb,
                        msg_if = self.msg_if)

    def systemStatusCb(self, msg):
        self.active_nodes = msg.active_nodes
        self.active_topics = msg.active_topics
        self.active_topic_types = msg.active_topic_types
        self.active_services = msg.active_services

    def initCb(self, do_updates = False):
        if self.node_if is not None:
            self.max_move_m = self.node_if.get_param(self.node_if_prefix + 'max_move_m')

            image_controls_dict = self.node_if.get_param(self.node_if_prefix + 'image_controls')
            if isinstance(image_controls_dict, dict):
                for key in self.image_controls_dict.keys():
                    if key in image_controls_dict.keys():
                        self.image_controls_dict[key] = image_controls_dict[key]
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
    # Source Management

    def updaterCb(self, timer):
        self.updateRobotState()
        self.updateImageSelection()
        self.updateFoundFlags()

        nepi_sdk.start_timer_process((1.0), self.updaterCb, oneshot = True)

    def updateRobotState(self):
        if self.rbx_if is None:
            return
        selected_topic = self.rbx_if.get_selected_topic()
        if selected_topic is None or selected_topic == '':
            selected_topic = 'None'
        self.rbx_namespace = selected_topic
        self.rbx_connected = self.rbx_if.check_connection()
        self.rbx_ready = self.rbx_connected and (self.rbx_if.check_ready() == True)

    def updateImageSelection(self):
        if self.image_if is None:
            return
        selected_topic = self.image_if.get_selected_topic()
        if selected_topic is None or selected_topic == '':
            selected_topic = 'None'

        if selected_topic != self.image_topic:
            self.msg_if.pub_info("Image selection changed to: " + str(selected_topic), log_name_list = self.log_name_list)
            self.image_topic = selected_topic
            self.resolveCompanionTopics()
            self.subscribeSourceTopics()
            # A new source invalidates everything derived from the old one.
            self.clearSourceData()

        self.image_connected = self.image_if.check_connection()

    def resolveCompanionTopics(self):
        # Sibling data products share a parent namespace. Prefer what the
        # selected image reports on its own ImageStatus -- the source is the
        # authority on its own siblings -- and fall back to the namespace join
        # when no status has arrived yet.
        if self.image_topic == 'None':
            self.depth_map_topic = ''
            self.depth_map_image_topic = ''
            self.overlay_image_topic = ''
            return

        depth_map_topic = ''
        status_dict = self.image_status_dict
        if isinstance(status_dict, dict):
            depth_map_topic = status_dict.get('depth_map_topic', '')
        if depth_map_topic == '':
            depth_map_topic = nepi_sdk.create_namespace(os.path.dirname(self.image_topic), DEPTH_MAP_TOPIC)
        self.depth_map_topic = depth_map_topic

        # The colourized render nests UNDER the depth map, not beside it. Never
        # taken from DepthMapStatus.image_topic, which names the colour image.
        self.depth_map_image_topic = nepi_sdk.create_namespace(self.depth_map_topic, DEPTH_MAP_IMAGE_TOPIC)

        # Where the image pub node will publish this source's overlay, beside
        # the source image itself.
        self.overlay_image_topic = os.path.join(os.path.dirname(self.image_topic), OVERLAY_IMG_PRODUCT)

    def updateFoundFlags(self):
        cur_time = nepi_utils.get_time()

        # A companion counts as found while it is publishing. The depth map
        # image is checked by topic discovery rather than by subscription: this
        # node never subscribes to it, the image pub node does.
        self.depth_map_found = (self.depth_map_topic != '' and
                                (cur_time - self.depth_map_last_time) < CONNECTED_TIMEOUT_SEC)

        found_image = ''
        if self.depth_map_image_topic != '':
            found_image = nepi_sdk.find_topic(self.depth_map_image_topic,
                                              exact = True,
                                              topics_list = self.active_topics,
                                              types_list = self.active_topic_types)
        self.depth_map_image_found = (found_image != '')

        self.objects_found = (cur_time - self.objects_last_time) < CONNECTED_TIMEOUT_SEC
        if self.objects_found == False:
            self.objects_list = []

        self.targets_found = (cur_time - self.targets_last_time) < CONNECTED_TIMEOUT_SEC
        if self.targets_found == False:
            self.targets_list = []

        self.obstacles_found = (HAS_OBSTACLES_MSG == True and
                                (cur_time - self.obstacles_last_time) < CONNECTED_TIMEOUT_SEC)
        if self.obstacles_found == False:
            self.obstacles_list = []

    def subscribeSourceTopics(self):
        # One subscriber set per selected image, rebuilt whenever the selection
        # changes. Only the depth map is subscribed here: the depth map image is
        # the image pub node's business, and the objects/targets/obstacles lists
        # arrive on the collective fan-out topics this class already subscribes
        # to for every source.
        self.unsubscribeSourceTopics()

        if self.image_topic == 'None' or self.depth_map_topic == '':
            return

        source_subs_dict = {
            'auto_move_depth_map_sub': {
                    'namespace': self.depth_map_topic,
                    'msg': Image,
                    'topic': '',
                    'qsize': 1,
                    'callback': self.depthMapCb,
                    'callback_args': ()
            },
            'auto_move_depth_map_status_sub': {
                    'namespace': self.depth_map_topic,
                    'msg': DepthMapStatus,
                    'topic': 'status',
                    'qsize': 1,
                    'callback': self.depthMapStatusCb,
                    'callback_args': ()
            },
        }

        self.msg_if.pub_info('Registering to depth map topic: ' + str(self.depth_map_topic), log_name_list = self.log_name_list)
        self.source_subs_if = NodeSubscribersIF(
                subs_dict = source_subs_dict,
                log_name_list = self.log_name_list,
                msg_if = self.msg_if)
        self.subscribed_image_topic = self.image_topic

    def unsubscribeSourceTopics(self):
        if self.source_subs_if is not None:
            try:
                self.source_subs_if.unregister_subs()
            except Exception as e:
                self.msg_if.pub_warn("Failed to unregister source subs: " + str(e), log_name_list = self.log_name_list)
            self.source_subs_if = None
        self.subscribed_image_topic = 'None'

    def clearSourceData(self):
        self.depth_map_lock.acquire()
        self.depth_map_slot = None
        self.depth_map_stamp = 0
        self.depth_map_lock.release()

        self.depth_map_status_dict = None
        self.depth_map_last_time = 0
        self.objects_list = []
        self.targets_list = []
        self.obstacles_list = []
        self.objects_last_time = 0
        self.targets_last_time = 0
        self.obstacles_last_time = 0
        self.objects_topic = ''
        self.targets_topic = ''
        self.obstacles_topic = ''

    ##########################################
    # Data Callbacks

    def imageStatusCb(self, status_dict):
        # ConnectDataIF hands the ImageStatus over as a dict.
        if not isinstance(status_dict, dict):
            return
        self.image_status_dict = status_dict
        self.image_width_px = status_dict.get('width_px', 0)
        self.image_height_px = status_dict.get('height_px', 0)

        # The reported depth map topic can arrive after the selection, so the
        # resolution is re-run on status rather than only on selection change.
        last_depth_map_topic = self.depth_map_topic
        self.resolveCompanionTopics()
        if self.depth_map_topic != last_depth_map_topic:
            self.subscribeSourceTopics()

    def depthMapStatusCb(self, status_msg):
        self.depth_map_status_dict = nepi_sdk.convert_msg2dict(status_msg)

    def depthMapCb(self, img_msg):
        self.depth_map_last_time = nepi_utils.get_time()
        try:
            np_depth_map = nepi_img.rosimg_to_cv2img(img_msg)
        except Exception as e:
            self.msg_if.pub_warn("Failed to convert depth map: " + str(e), log_name_list = self.log_name_list, throttle_s = 5.0)
            return

        self.depth_map_lock.acquire()
        self.depth_map_slot = np_depth_map
        self.depth_map_stamp = float(img_msg.header.stamp.to_sec())
        self.depth_map_lock.release()

    def detectionsCb(self, msg):
        # The fan-out carries every detector on the device. Only the messages
        # computed from the selected image are ours.
        if self.image_topic == 'None' or msg.source_topic != self.image_topic:
            return
        self.objects_last_time = nepi_utils.get_time()
        self.objects_topic = nepi_sdk.create_namespace(msg.process_namespace, DETECTIONS_ALL_TOPIC)
        objects_list = []
        for detection_msg in msg.detections:
            objects_list.append(nepi_sdk.convert_msg2dict(detection_msg))
        self.objects_list = objects_list

    def targetsCb(self, msg):
        if self.image_topic == 'None' or msg.source_topic != self.image_topic:
            return
        self.targets_last_time = nepi_utils.get_time()
        self.targets_topic = nepi_sdk.create_namespace(msg.process_namespace, TARGETS_ALL_TOPIC)
        targets_list = []
        for target_msg in msg.targets:
            targets_list.append(nepi_sdk.convert_msg2dict(target_msg))
        self.targets_list = targets_list

    def obstaclesCb(self, msg):
        # Obstacles match on the DEPTH MAP topic, not the image topic: an
        # obstacles process consumes depth maps, so that is what its
        # source_topic names.
        if self.depth_map_topic == '' or msg.source_topic != self.depth_map_topic:
            return
        self.obstacles_last_time = nepi_utils.get_time()
        self.obstacles_topic = nepi_sdk.create_namespace(msg.process_namespace, OBSTACLES_ALL_TOPIC)
        obstacles_list = []
        for obstacle_msg in msg.obstacles:
            obstacles_list.append(nepi_sdk.convert_msg2dict(obstacle_msg))
        self.obstacles_list = obstacles_list

    ##########################################
    # Command Callbacks

    def setGotoXCb(self, msg):
        self.goto_x_m = float(msg.data)
        self.goto_clamped = False
        self.publish_status()

    def setGotoYCb(self, msg):
        self.goto_y_m = float(msg.data)
        self.goto_clamped = False
        self.publish_status()

    def setGotoZCb(self, msg):
        self.goto_z_m = float(msg.data)
        self.goto_clamped = False
        self.publish_status()

    def setMaxMoveCb(self, msg):
        max_move_m = msg.data
        if max_move_m < MIN_MAX_MOVE_M:
            max_move_m = MIN_MAX_MOVE_M
        elif max_move_m > MAX_MAX_MOVE_M:
            max_move_m = MAX_MAX_MOVE_M
        self.max_move_m = max_move_m
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix + 'max_move_m', self.max_move_m)
            self.save_config()

    def setDepthMapTransparencyCb(self, msg):
        self.setImageRatio('depth_map_transparency', msg.data)

    def setShowDepthMapCb(self, msg):
        self.setImageControl('show_depth_map_enabled', msg.data)

    def setShowObjectsCb(self, msg):
        self.setImageControl('show_objects_enabled', msg.data)

    def setShowTargetsCb(self, msg):
        self.setImageControl('show_targets_enabled', msg.data)

    def setShowObstaclesCb(self, msg):
        self.setImageControl('show_obstacles_enabled', msg.data)

    def setShowCrosshairCb(self, msg):
        self.setImageControl('show_crosshair_enabled', msg.data)

    def setImageControl(self, control_name, enabled):
        if control_name not in self.image_controls_dict.keys():
            return
        self.setImageControlValue(control_name, (enabled == True))

    def setImageRatio(self, control_name, ratio):
        if control_name not in self.image_controls_dict.keys():
            return
        try:
            ratio = float(ratio)
        except (TypeError, ValueError):
            return
        if ratio < 0.0:
            ratio = 0.0
        elif ratio > 1.0:
            ratio = 1.0
        self.setImageControlValue(control_name, ratio)

    def setImageControlValue(self, control_name, value):
        self.image_controls_dict[control_name] = value
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix + 'image_controls', self.image_controls_dict)
            self.save_config()

    ##########################################
    # Click to XYZ

    def mouseEventCb(self, msg):
        # The viewer publishes drag, window and scroll events on this same
        # topic. Only a click sets a goto point.
        if msg.click_event == False:
            return
        self.applyClick(msg)

    def applyClick(self, msg):
        click_x = int(msg.click.x)
        click_y = int(msg.click.y)

        # The viewer reports IMAGE pixels, not canvas pixels: Nepi_IF_ImageViewer
        # sizes its canvas to the streamed frame and scales the browser event
        # into that raster before publishing. It is still not necessarily the
        # DEPTH MAP raster -- the overlay image and the depth map are separate
        # products and need not share a resolution -- so the click is carried as
        # a ratio of the viewed image and re-expressed in whatever raster it is
        # being applied to. Width and height come off the ImageStatus the viewer
        # attached to the event, which is the authority on the frame clicked.
        width_px = 0
        height_px = 0
        try:
            width_px = int(msg.image_status_msg.width_px)
            height_px = int(msg.image_status_msg.height_px)
        except Exception:
            width_px = 0
            height_px = 0
        if width_px <= 0 or height_px <= 0:
            width_px = int(self.image_width_px)
            height_px = int(self.image_height_px)
        if width_px <= 0 or height_px <= 0:
            self.click_msg = 'Click ignored: image size unknown'
            self.publish_status()
            return

        x_ratio = float(click_x) / float(width_px)
        y_ratio = float(click_y) / float(height_px)
        x_ratio = min(max(x_ratio, 0.0), 1.0)
        y_ratio = min(max(y_ratio, 0.0), 1.0)

        self.has_click = True
        self.click_pixel_x = click_x
        self.click_pixel_y = click_y
        self.click_x_ratio = x_ratio
        self.click_y_ratio = y_ratio

        self.depth_map_lock.acquire()
        np_depth_map = self.depth_map_slot
        self.depth_map_lock.release()

        if np_depth_map is None:
            self.click_depth_valid = False
            self.click_msg = 'No depth map for this image, goto values unchanged'
            self.publish_status()
            return

        range_m = self.getRangeAtRatio(np_depth_map, x_ratio, y_ratio)
        if range_m is None:
            self.click_depth_valid = False
            self.click_msg = 'No depth return at that pixel, goto values unchanged'
            self.publish_status()
            return

        [width_deg, height_deg] = self.getSourceFovDeg()

        # Bearing off the image centre. Azimuth is positive to the right of
        # centre, elevation positive above it -- image rows run top down, hence
        # the sign flip on y.
        azimuth_deg = (x_ratio - 0.5) * width_deg
        elevation_deg = (0.5 - y_ratio) * height_deg
        azimuth_rad = math.radians(azimuth_deg)
        elevation_rad = math.radians(elevation_deg)

        # Body frame per nepi_interfaces/GotoPosition: x forward, y LEFT, z up,
        # in meters. A positive azimuth is to the right, so y takes the negative.
        x_m = range_m * math.cos(elevation_rad) * math.cos(azimuth_rad)
        y_m = -range_m * math.cos(elevation_rad) * math.sin(azimuth_rad)
        z_m = range_m * math.sin(elevation_rad)

        clamped = False
        max_move_m = float(self.max_move_m)
        if x_m > max_move_m and x_m > 0.0:
            # Scale the whole vector rather than truncating x alone. Truncating
            # x would swing the commanded bearing away from the point the
            # operator clicked; scaling stops short along the same line.
            scale = max_move_m / x_m
            x_m = x_m * scale
            y_m = y_m * scale
            z_m = z_m * scale
            clamped = True

        self.goto_x_m = x_m
        self.goto_y_m = y_m
        self.goto_z_m = z_m
        self.goto_clamped = clamped
        self.click_depth_valid = True

        click_msg = ('Click at ' + str(click_x) + ',' + str(click_y) +
                     ' -> ' + str(round(range_m, 2)) + 'm')
        if clamped == True:
            click_msg = click_msg + ', clamped to ' + str(round(max_move_m, 2)) + 'm'
        self.click_msg = click_msg

        self.publish_status()

    def getSourceFovDeg(self):
        # The depth map's own status is the geometry authority for depth data.
        # The image status is the fallback, and the module defaults the last
        # resort -- the same fallback chain nepi_obstacles uses.
        width_deg = 0.0
        height_deg = 0.0
        if isinstance(self.depth_map_status_dict, dict):
            width_deg = self.depth_map_status_dict.get('width_deg', 0.0)
            height_deg = self.depth_map_status_dict.get('height_deg', 0.0)
        if (width_deg is None or width_deg <= 0.0) and isinstance(self.image_status_dict, dict):
            width_deg = self.image_status_dict.get('width_deg', 0.0)
        if (height_deg is None or height_deg <= 0.0) and isinstance(self.image_status_dict, dict):
            height_deg = self.image_status_dict.get('height_deg', 0.0)
        if width_deg is None or width_deg <= 0.0:
            width_deg = DEFAULT_WIDTH_DEG
        if height_deg is None or height_deg <= 0.0:
            height_deg = DEFAULT_HEIGHT_DEG
        return [float(width_deg), float(height_deg)]

    def getRangeAtRatio(self, np_depth_map, x_ratio, y_ratio):
        # Returns range in METERS, or None when the patch holds no usable
        # return.
        try:
            shape = np_depth_map.shape
            height_px = int(shape[0])
            width_px = int(shape[1])
            if width_px <= 0 or height_px <= 0:
                return None

            x_px = int(round(x_ratio * (width_px - 1)))
            y_px = int(round(y_ratio * (height_px - 1)))
            x_px = min(max(x_px, 0), width_px - 1)
            y_px = min(max(y_px, 0), height_px - 1)

            x_min = max(x_px - DEPTH_SAMPLE_HALF_PX, 0)
            x_max = min(x_px + DEPTH_SAMPLE_HALF_PX + 1, width_px)
            y_min = max(y_px - DEPTH_SAMPLE_HALF_PX, 0)
            y_max = min(y_px + DEPTH_SAMPLE_HALF_PX + 1, height_px)

            patch = np.asarray(np_depth_map[y_min:y_max, x_min:x_max], dtype = np.float64)
            valid = patch[np.isfinite(patch)]
            valid = valid[valid > 0.0]
            if valid.size == 0:
                return None

            # UNIT CONVERSION: NEPI depth map arrays carry MILLIMETRE range
            # values, while every range bound on their status message and every
            # goto value on the wire is in METRES. nepi_img is the authority on
            # this split -- get_range_from_npDepthMap divides by 1000 to report
            # metres. Divide here, once, at the only place this app reads a raw
            # depth map.
            range_mm = float(np.median(valid))
            range_m = range_mm / 1000.0
            return range_m
        except Exception as e:
            self.msg_if.pub_warn("Failed to sample depth map: " + str(e), log_name_list = self.log_name_list, throttle_s = 5.0)
            return None

    ##########################################
    # Goto Process

    def gotoTriggerCb(self, msg):
        self.startGoto()

    def gotoCancelCb(self, msg):
        self.cancelGoto()

    def startGoto(self):
        if self.goto_state == NepiAppAutoMoveStatus.GOTO_STATE_PLANNING or \
                self.goto_state == NepiAppAutoMoveStatus.GOTO_STATE_MOVING:
            self.msg_if.pub_warn("Goto already running", log_name_list = self.log_name_list)
            return
        if self.rbx_if is None or self.rbx_connected == False:
            self.goto_state = NepiAppAutoMoveStatus.GOTO_STATE_IDLE
            self.goto_msg = 'No robot connected'
            self.msg_if.pub_warn("Goto ignored: no robot connected", log_name_list = self.log_name_list)
            self.publish_status()
            return

        self.goto_cancel_requested = False
        self.goto_plan = []
        self.goto_step = 0
        self.goto_step_count = 0
        self.goto_step_issued = False
        self.goto_saw_busy = False
        self.goto_state = NepiAppAutoMoveStatus.GOTO_STATE_PLANNING
        self.goto_msg = 'Planning move'
        self.publish_status()

    def cancelGoto(self):
        self.goto_cancel_requested = True
        if self.rbx_if is not None and self.rbx_connected == True:
            try:
                self.rbx_if.go_stop()
            except Exception as e:
                self.msg_if.pub_warn("Failed to send stop: " + str(e), log_name_list = self.log_name_list)
        self.goto_plan = []
        self.goto_step = 0
        self.goto_step_count = 0
        self.goto_step_issued = False
        self.goto_state = NepiAppAutoMoveStatus.GOTO_STATE_CANCELLED
        self.goto_msg = 'Cancelled by operator'
        self.publish_status()

    def gotoProcessCb(self, timer):
        try:
            if self.goto_state == NepiAppAutoMoveStatus.GOTO_STATE_PLANNING:
                self.runPlanning()
            elif self.goto_state == NepiAppAutoMoveStatus.GOTO_STATE_MOVING:
                self.runMoving()
        except Exception as e:
            self.msg_if.pub_warn("Goto process failed: " + str(e), log_name_list = self.log_name_list, throttle_s = 5.0)
            self.goto_state = NepiAppAutoMoveStatus.GOTO_STATE_IDLE
            self.goto_msg = 'Goto process failed: ' + str(e)
            self.publish_status()

        nepi_sdk.start_timer_process((0.2), self.gotoProcessCb, oneshot = True)

    def runPlanning(self):
        if self.goto_cancel_requested == True:
            self.cancelGoto()
            return

        goto_dict = {
            'x_m': float(self.goto_x_m),
            'y_m': float(self.goto_y_m),
            'z_m': float(self.goto_z_m),
            'max_move_m': float(self.max_move_m),
        }

        self.depth_map_lock.acquire()
        np_depth_map = self.depth_map_slot
        self.depth_map_lock.release()

        robot_dict = self.getRobotDict()
        controls_dict = self.get_controls_dict()

        plan = self.planMove(goto_dict,
                             np_depth_map,
                             copy.deepcopy(self.objects_list),
                             copy.deepcopy(self.targets_list),
                             robot_dict,
                             controls_dict,
                             copy.deepcopy(self.obstacles_list))

        if plan is None or len(plan) == 0:
            self.goto_state = NepiAppAutoMoveStatus.GOTO_STATE_COMPLETE
            self.goto_msg = 'Planner returned no moves'
            self.publish_status()
            return

        self.goto_plan = plan
        self.goto_step = 0
        self.goto_step_count = len(plan)
        self.goto_step_issued = False
        self.goto_saw_busy = False
        self.goto_state = NepiAppAutoMoveStatus.GOTO_STATE_MOVING
        self.goto_msg = 'Executing move 1 of ' + str(self.goto_step_count)
        self.publish_status()

    def runMoving(self):
        if self.goto_cancel_requested == True:
            self.cancelGoto()
            return

        if self.rbx_if is None or self.rbx_connected == False:
            self.goto_state = NepiAppAutoMoveStatus.GOTO_STATE_IDLE
            self.goto_msg = 'Lost robot connection during move'
            self.publish_status()
            return

        if self.goto_step >= self.goto_step_count:
            self.goto_state = NepiAppAutoMoveStatus.GOTO_STATE_COMPLETE
            self.goto_msg = 'Move complete'
            self.publish_status()
            return

        step_dict = self.goto_plan[self.goto_step]

        if self.goto_step_issued == False:
            # A relative body-frame move, in METERS. The values already carry
            # the units the RBX interface expects, so nothing is converted here.
            self.rbx_if.goto_position(float(step_dict.get('x_m', 0.0)),
                                      float(step_dict.get('y_m', 0.0)),
                                      float(step_dict.get('z_m', 0.0)),
                                      float(step_dict.get('yaw_deg', 0.0)))
            self.goto_step_issued = True
            self.goto_saw_busy = False
            self.goto_step_start_time = nepi_utils.get_time()
            self.goto_msg = ('Executing move ' + str(self.goto_step + 1) + ' of ' +
                             str(self.goto_step_count) + ': ' + str(step_dict.get('description', '')))
            self.publish_status()
            return

        elapsed = nepi_utils.get_time() - self.goto_step_start_time
        device_ready = (self.rbx_if.check_ready() == True)

        if device_ready == False:
            # Busy is what confirms the device accepted the command.
            self.goto_saw_busy = True

        step_done = False
        if self.goto_saw_busy == True and device_ready == True:
            step_done = True
        elif self.goto_saw_busy == False and elapsed > GOTO_BUSY_WAIT_SEC and device_ready == True:
            # The device never reported busy. A move short enough to finish
            # inside one status period looks exactly like this, so it is treated
            # as done rather than left to time out.
            step_done = True
        elif elapsed > GOTO_STEP_TIMEOUT_SEC:
            self.goto_state = NepiAppAutoMoveStatus.GOTO_STATE_IDLE
            self.goto_msg = 'Move step ' + str(self.goto_step + 1) + ' timed out'
            self.publish_status()
            return

        if step_done == True:
            self.goto_step = self.goto_step + 1
            self.goto_step_issued = False
            self.goto_saw_busy = False
            if self.goto_step >= self.goto_step_count:
                self.goto_state = NepiAppAutoMoveStatus.GOTO_STATE_COMPLETE
                self.goto_msg = 'Move complete'
            self.publish_status()

    def getRobotDict(self):
        # What the planner is told about the robot. Kept to what the RBX connect
        # interface reports so the planner never has to talk to ROS itself.
        robot_dict = {
            'namespace': self.rbx_namespace,
            'connected': self.rbx_connected,
            'ready': self.rbx_ready,
            'status_dict': None,
        }
        if self.rbx_if is not None and self.rbx_connected == True:
            try:
                robot_dict['status_dict'] = self.rbx_if.get_status_dict()
            except Exception:
                robot_dict['status_dict'] = None
        return robot_dict

    ##########################################
    # Status

    def publishStatusCb(self, timer):
        self.publish_status()
