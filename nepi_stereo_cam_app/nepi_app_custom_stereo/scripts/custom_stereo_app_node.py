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
# Redistributions in source code must retain this top-level comment bstab.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com

"""Custom Stereo Depth app node (OUTLINE / SKELETON).

Assembled from the NEPI app template plus the developer-authored
idx_custom_stereo_node.py logic. It brings up:

  * TWO ConnectIDXDeviceIF selectors (left + right camera), each owning its own
    connect namespace (<node>/left_cam_connect and <node>/right_cam_connect),
    following nepi_app_idx_connect. The RUI drives selection through the
    Nepi_IF_ConnectIDX component.
  * A "processes" dropdown (available_processes / selected_process +
    set_selected_process / reload_processes), sourced from stereo_settings
    (create/update_processes_dict), following nepi_app_pan_tilt_auto.
  * An IDXDeviceIF that reports the depth_map data product, using the
    developer's getDepthMap logic.

WHAT IS DEFERRED (clearly marked TODOs for the next developer):
  * The real per-frame path: subscribing to the two SELECTED camera image
    topics, converting to cv2 frames, rectifying, and feeding them into
    compute_depth_map. Right now grabPair() is a marked skeleton.
  * Real stereo calibration (calibrate.py) and the calib .npz path.
  * Filling out device_info / capSettings / factorySettings for the IDX IF.
"""

import time
import copy
import importlib

import numpy as np
import cv2

from std_msgs.msg import String, Empty, Float32

from nepi_interfaces.msg import UpdateFloat

from nepi_app_custom_stereo.msg import NepiAppCustomStereoStatus

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils

from nepi_api.node_if import NodeClassIF
from nepi_api.messages_if import MsgIF

# NEPI IDX device interfaces
from nepi_api.device_if_idx import IDXDeviceIF
from nepi_api.connect_device_if_idx import ConnectIDXDeviceIF

# App-local sibling modules (co-located in scripts/). Bare imports -- the
# developer's original node used package-qualified names for the wrong package
# (nepi_idx_custom_stereo.*); reconciled here to bare sibling imports, matching
# stereo_settings.py / calibrate.py.
from calibrate import Rectifier
import stereo_settings


#########################################
# Control Values
STATUS_PUBLISH_RATE_HZ = 1.0
UPDATE_RATE_HZ = 1.0

# IDX data product subtopics.
#
# The color subtopic is joined onto a SELECTED camera's namespace, which for an
# IDX connect is already the device's '<device>/idx' namespace (ConnectIDXDeviceIF
# get_selected_topic() -> selected_topic, and the connect subscribes to
# selected_topic + '/status' for DeviceIDXStatus, published at <device>/idx/status).
# So the color image resolves to <device>/idx/color_image -- the subtopic here must
# NOT re-include 'idx' or the topic doubles up to <device>/idx/idx/color_image.
IDX_COLOR_SUBTOPIC = 'color_image'
# The depth subtopic is joined onto THIS app node's namespace (not a selected
# device), and our own IDX device publishes under <node>/idx, so it keeps 'idx/'.
IDX_DEPTH_SUBTOPIC = 'idx/depth_map'

#########################################
# Node Class
#########################################

class NepiCustomStereoApp(object):

    DEFAULT_NODE_NAME = "app_custom_stereo"
    DEFAULT_MAX_FRAMERATE = 10.0

    # Data products this app reports through the IDX interface.
    data_products = ["depth_map"]

    node_if = None
    idx_if = None
    left_cam_connect_if = None
    right_cam_connect_if = None
    msg_if = None

    # Processes (nepi stab/auto "processes" pattern; see stereo_settings.py)
    available_processes = list(stereo_settings.PROCESSES_DICT.keys())
    selected_process = stereo_settings.DEFAULT_PROCESS
    processes_dict = stereo_settings.create_processes_dict()
    process_ready = True

    # Rectification (calibration is offline; see calibrate.py)
    rectifier = None
    calib_path = None

    # Framerate throttle + runtime data
    max_framerate = DEFAULT_MAX_FRAMERATE
    dm_data_last_time = None
    stereo_data_dict = stereo_settings.get_blank_data_dict()

    # Latest computed depth map (cache). The app loop (updaterCb) computes depth
    # and stores it here; the IDX getDepthMap callback just serves it. Stays None
    # until the per-frame wiring produces a real depth map -- which is also the
    # gate for standing up the IDX device interface.
    depth_map = None
    depth_map_timestamp = None

    # Selected camera frames (populated by the per-frame wiring -- TODO skeleton)
    left_frame = None
    right_frame = None

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
        self.available_processes = list(stereo_settings.PROCESSES_DICT.keys())
        self.selected_process = stereo_settings.DEFAULT_PROCESS
        self.processes_dict = stereo_settings.create_processes_dict()
        self.stereo_data_dict = stereo_settings.get_blank_data_dict()
        self.max_framerate = self.DEFAULT_MAX_FRAMERATE

        # ---- Rectification ----
        # TODO: point calib_path at the real stereo calibration .npz produced by
        # calibrate.calibrate(); when present, push the true geometry into every
        # process's settings (developer's original loop, kept for the next dev):
        #   self.rectifier = Rectifier(self.calib_path)
        #   for proc in self.processes_dict.values():
        #       proc["focal_length_px"] = self.rectifier.focal_length_px
        #       proc["baseline_mm"] = self.rectifier.baseline_mm
        self.rectifier = None

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
            'selected_process': {
                'namespace': self.node_namespace,
                'factory_val': self.selected_process
            },
            'processes_dict': {
                'namespace': self.node_namespace,
                'factory_val': self.processes_dict
            }
        }

        # Publishers Config Dict ####################
        self.PUBS_DICT = {
            'status_pub': {
                'namespace': self.node_namespace,
                'topic': 'status',
                'msg': NepiAppCustomStereoStatus,
                'qsize': 1,
                'latch': True
            }
        }

        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            'set_selected_process': {
                'namespace': self.node_namespace,
                'topic': 'set_selected_process',
                'msg': String,
                'qsize': 10,
                'callback': self.setSelectedProcessCb,
                'callback_args': ()
            },
            'reload_processes': {
                'namespace': self.node_namespace,
                'topic': 'reload_processes',
                'msg': Empty,
                'qsize': 10,
                'callback': self.reloadProcessesCb,
                'callback_args': ()
            },
            'set_process_control_value': {
                'namespace': self.node_namespace,
                'topic': 'set_process_control_value',
                'msg': UpdateFloat,
                'qsize': 10,
                'callback': self.setProcessControlValueCb,
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
        # Left + Right IDX camera selectors.
        # Two distinct connect namespaces so each dropdown selects its own device
        # (mirrors nepi_app_idx_connect, one ConnectIDXDeviceIF per camera). The
        # Nepi_IF_ConnectIDX RUI components talk to these connect namespaces.
        self.left_cam_connect_if = ConnectIDXDeviceIF(
            connect_name='left_cam_connect',
            msg_if=self.msg_if
        )
        self.left_cam_connect_if.wait_for_connect_ready()

        self.right_cam_connect_if = ConnectIDXDeviceIF(
            connect_name='right_cam_connect',
            msg_if=self.msg_if
        )
        self.right_cam_connect_if.wait_for_connect_ready()

        ##############################
        # NOTE: the IDX device interface is NOT created here. Registering it
        # immediately would advertise this app as an IDX device (so it self-lists
        # in the left/right camera dropdowns) and expose the depth_map product
        # before any depth map exists. Instead createIdxIf() is called lazily from
        # updaterCb() the first time a real depth map is produced -- so the app and
        # its depth_map only appear once there is actual depth data to publish.

        ##############################
        self.initCb(do_updates=True)

        time.sleep(1)
        nepi_sdk.start_timer_process(float(1) / UPDATE_RATE_HZ, self.updaterCb, oneshot=True)
        nepi_sdk.start_timer_process(float(1) / STATUS_PUBLISH_RATE_HZ, self.statusPublishCb)

        time.sleep(1)
        self.msg_if.pub_info("Initialization Complete")

        nepi_sdk.on_shutdown(self.cleanup_actions)
        nepi_sdk.spin()


    ###################
    ## IDX interface bring-up

    def createIdxIf(self):
        # TODO: fill device_info / capSettings / factorySettings from the real
        # stereo rig. device_info is required by IDXDeviceIF; these are skeleton
        # placeholders so the interface registers and reports depth_map.
        device_info = {
            'device_name': self.node_name,
            'path': self.node_namespace,
            'serial_number': 'TODO',
            'hw_version': 'TODO',
            'sw_version': '0.0.0'
        }
        # IDX device-settings channel (SettingsIF). This is SEPARATE from the
        # "processes" dropdown, which the app node handles itself via its own
        # params (selected_process / processes_dict) and the set_selected_process
        # / reload_processes subscribers. SettingsIF expects a settings dict of
        # {name: {'name','type','value'}} entries and iterates them at init, so
        # the processes_dict (nested process definitions, no 'name' key) must NOT
        # be handed to it -- doing so raises KeyError: 'name'. Until the next
        # developer exposes real device settings (resolution, framerate, gain,
        # ...), these are empty so the interface registers cleanly.
        self.idx_if = IDXDeviceIF(
            device_info,
            data_products=self.data_products,
            capSettings=self.getCapSettings(),
            factorySettings=dict(),
            getSettingsFunction=self.getIdxSettings,
            settingUpdateFunction=self.updateIdxSetting,
            setMaxFramerate=self.setMaxFramerate,
            getFramerate=self.getFramerate,
            getDepthMap=self.getDepthMap,
            stopDepthMapAcquisition=self.stopDepthMap,
            msg_if=self.msg_if
            # TODO: getColorImage, getNavPoseCb, ...
        )

    def getCapSettings(self):
        # IDX device-settings capabilities. Empty skeleton -- return the same
        # {name: {'name','type','options'}} shape real IDX drivers build (see
        # idx_v4l2_node.getCapSettings) once real device controls are exposed.
        return dict()

    def getIdxSettings(self):
        # IDX device-settings read path (getSettingsFunction). Returns a settings
        # dict of {name: {'name','type','value'}} entries; empty for the skeleton.
        # NOTE: this is NOT the processes dict -- see setSelectedProcess().
        return dict()

    def updateIdxSetting(self, setting):
        # IDX device-settings write path (settingUpdateFunction). No device
        # settings are exposed yet, so accept and no-op. Returns (success, msg).
        return True, "no device settings exposed"


    ###################
    ## Processes settings back-and-forth with the RUI (IDX contract callbacks)

    def getSettings(self):
        # RUI read path: hand out the current nested processes settings.
        return self.processes_dict

    def settingUpdateFunction(self, incoming_processes_dict):
        # RUI write path: sanitize incoming settings against known defaults.
        # update_processes_dict drops unknown keys and preserves structure, so a
        # malformed RUI payload can't inject bad settings.
        self.processes_dict = stereo_settings.update_processes_dict(incoming_processes_dict)
        if self.node_if is not None:
            self.node_if.set_param('processes_dict', self.processes_dict)
        return True, "stereo settings updated"

    def setSelectedProcess(self, process_name):
        if process_name in stereo_settings.PROCESSES_DICT:
            self.selected_process = process_name
            return True
        return False

    def setMaxFramerate(self, max_framerate):
        self.max_framerate = max_framerate

    def getFramerate(self):
        return self.max_framerate


    ###################
    ## Frame acquisition + depth map (developer's idx_custom_stereo_node logic)

    def grabPair(self):
        # Return (ok, left_bgr, right_bgr). Frames must be time-synced.
        #
        # TODO (NEXT DEVELOPER -- THE CORE WIRING): this is the per-frame path.
        # Subscribe to the two SELECTED camera image topics (see
        # computeImageTopics()/updaterCb), convert the latest sensor_msgs/Image
        # from each into cv2 BGR frames, and return them here. If the cameras are
        # not hardware-synced, depth on moving scenes will be wrong.
        #   left = self.left_frame
        #   right = self.right_frame
        #   if left is None or right is None:
        #       return False, None, None
        #   return True, left, right
        return False, None, None

    def updateDepthMap(self):
        # Compute a depth map and cache it. Called from the app's own updaterCb
        # loop (NOT the IDX callback) so that depth can be produced BEFORE the IDX
        # device interface exists -- the first successful compute is what triggers
        # createIdxIf(). Returns True if a new depth map was produced.
        #
        # framerate throttle
        last_time = self.dm_data_last_time
        current_time = nepi_utils.get_time()
        if last_time is not None:
            fr_delay = float(1) / self.max_framerate
            if (current_time - last_time) < fr_delay:
                return False

        # Grab a synchronized L/R pair (TODO skeleton -- returns no frames yet).
        # Until the per-frame wiring is implemented this returns False, so no
        # depth map is ever produced and the IDX device is never brought up.
        ok, left, right = self.grabPair()
        if not ok:
            return False

        # Rectify (compute_depth_map assumes rectified input)
        if self.rectifier is not None:
            left, right = self.rectifier.rectify(left, right)

        # Run the selected stereo process (fills self.stereo_data_dict)
        process_fn = stereo_settings.PROCESSES_DICT[self.selected_process]['process_function']
        settings = self.processes_dict[self.selected_process]
        self.stereo_data_dict, _ = process_fn(left, right, self.stereo_data_dict, settings)

        np_depth_map = self.stereo_data_dict['depth_map']   # (H,W) float32 mm
        # Match the ZED stereo convention: invalid pixels -> nan instead of 0.0.
        np_depth_map[np_depth_map <= 0.0] = np.nan

        self.depth_map = np_depth_map
        self.depth_map_timestamp = nepi_utils.get_time()
        self.dm_data_last_time = nepi_utils.get_time()
        return True

    def getDepthMap(self):
        # IDX getDepthMap contract (mirrors idx_zed_node.getDepthMap). Serves the
        # latest depth map cached by updateDepthMap(). Returns
        # (status, msg, np_depth_map, timestamp, encoding).
        if self.depth_map is None:
            return False, "No depth map yet", None, None, None
        return True, "", self.depth_map, self.depth_map_timestamp, '32FC1'

    def stopDepthMap(self):
        # TODO: any teardown when the RUI stops the depth subscription.
        pass


    ###################
    ## App Callbacks

    def updaterCb(self, timer):
        # TODO (NEXT DEVELOPER): (re)wire the per-frame camera subscriptions here.
        # When the left/right selector picks a device, compute its color image
        # topic (computeImageTopics) and (re)subscribe an image callback that
        # decodes into self.left_frame / self.right_frame for grabPair(). Left as
        # a marked skeleton -- no subscription is created yet.

        # Try to produce a depth map from the current frame pair.
        self.updateDepthMap()

        # Bring up the IDX device interface the first time real depth exists. This
        # is deliberately lazy: until a depth map is produced, the app does not
        # register as an IDX device (so it does not self-list in the camera
        # dropdowns) and does not advertise the depth_map product.
        if self.idx_if is None and self.depth_map is not None:
            self.msg_if.pub_info("Depth map produced -- bringing up IDX device interface")
            self.createIdxIf()

        nepi_sdk.start_timer_process(float(1) / UPDATE_RATE_HZ, self.updaterCb, oneshot=True)

    def setSelectedProcessCb(self, msg):
        self.msg_if.pub_info(str(msg))
        process_name = msg.data
        if self.setSelectedProcess(process_name):
            self.publish_status()
            if self.node_if is not None:
                self.node_if.set_param('selected_process', self.selected_process)
                self.node_if.save_config()

    def setProcessControlValueCb(self, msg):
        # RUI edited one of the selected process's tunable settings. msg is an
        # UpdateFloat (name + value). Route the value to the matching key -- either
        # a top-level setting or one inside the nested stereo_controls_dict -- and
        # cast it back to the original key's type so ints/bools stay ints/bools.
        name = msg.name
        value = msg.value
        settings = self.processes_dict.get(self.selected_process, None)
        if settings is None:
            return
        updated = False
        if name in settings and name != 'stereo_controls_dict':
            settings[name] = self.castLikeExisting(settings[name], value)
            updated = True
        elif name in settings.get('stereo_controls_dict', {}):
            settings['stereo_controls_dict'][name] = self.castLikeExisting(
                settings['stereo_controls_dict'][name], value)
            updated = True
        if updated:
            # Sanitize (drops unknown keys, keeps structure) and persist.
            self.processes_dict = stereo_settings.update_processes_dict(self.processes_dict)
            if self.node_if is not None:
                self.node_if.set_param('processes_dict', self.processes_dict)
                self.node_if.save_config()
            self.publish_status()

    def castLikeExisting(self, existing, value):
        # Cast an incoming float back to the type of the existing value. bool must
        # be checked before int because bool is a subclass of int.
        if isinstance(existing, bool):
            return bool(round(value))
        if isinstance(existing, int):
            return int(round(value))
        return float(value)

    def flattenProcessControls(self):
        # Flatten the selected process's settings into index-aligned (names,
        # values) lists for the status message: top-level numeric settings first,
        # then the nested stereo_controls_dict. Non-numeric keys are skipped.
        names = []
        values = []
        settings = self.processes_dict.get(self.selected_process, {})
        for key in settings.keys():
            if key == 'stereo_controls_dict':
                continue
            val = settings[key]
            if isinstance(val, (int, float, bool)):
                names.append(key)
                values.append(float(val))
        for key, val in settings.get('stereo_controls_dict', {}).items():
            if isinstance(val, (int, float, bool)):
                names.append(key)
                values.append(float(val))
        return names, values

    def reloadProcessesCb(self, msg):
        # Reload the stereo_settings module so edits to the processes registry are
        # picked up without restarting the node (pattern from pan_tilt_auto
        # reloadAutosCb).
        self.process_ready = False
        nepi_sdk.sleep(1)
        try:
            importlib.reload(stereo_settings)
            self.processes_dict = stereo_settings.update_processes_dict(self.processes_dict)
            self.available_processes = list(stereo_settings.PROCESSES_DICT.keys())
            if self.selected_process not in self.available_processes:
                self.selected_process = self.available_processes[0]
            self.msg_if.pub_info("Stereo processes reloaded")
            self.process_ready = True
        except Exception as e:
            self.msg_if.pub_warn("Failed to reload stereo_settings module: " + str(e))
        self.publish_status()


    #######################
    ### Config Functions

    def initCb(self, do_updates=False):
        if self.node_if is not None:
            self.selected_process = self.node_if.get_param('selected_process')
            processes_dict = self.node_if.get_param('processes_dict')
            self.processes_dict = stereo_settings.update_processes_dict(processes_dict)
            if self.selected_process not in self.processes_dict.keys():
                self.selected_process = list(self.processes_dict.keys())[0]
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
    ## Status Publishers

    def statusPublishCb(self, timer):
        self.publish_status()

    def computeImageTopics(self):
        # Build the color image topic for each selected IDX device from its
        # selected device namespace. Returns (left_ns, left_img, right_ns,
        # right_img); 'None' where unselected. TODO: confirm IDX_COLOR_SUBTOPIC.
        left_ns = 'None'
        right_ns = 'None'
        if self.left_cam_connect_if is not None:
            left_ns = self.left_cam_connect_if.get_selected_topic()
        if self.right_cam_connect_if is not None:
            right_ns = self.right_cam_connect_if.get_selected_topic()

        left_img = 'None'
        right_img = 'None'
        if left_ns not in (None, 'None', ''):
            left_img = nepi_sdk.create_namespace(left_ns, IDX_COLOR_SUBTOPIC)
        else:
            left_ns = 'None'
        if right_ns not in (None, 'None', ''):
            right_img = nepi_sdk.create_namespace(right_ns, IDX_COLOR_SUBTOPIC)
        else:
            right_ns = 'None'
        return left_ns, left_img, right_ns, right_img

    def publish_status(self):
        """Publish the latched app status message with process + camera state."""
        status_msg = NepiAppCustomStereoStatus()

        # Processes dropdown
        status_msg.available_processes = self.available_processes
        status_msg.selected_process = self.selected_process
        status_msg.process_ready = self.process_ready

        # Selected process's tunable settings, flattened for the RUI editable boxes
        control_names, control_values = self.flattenProcessControls()
        status_msg.process_control_names = control_names
        status_msg.process_control_values = control_values

        # Left / right camera connect state + image topics
        left_ns, left_img, right_ns, right_img = self.computeImageTopics()
        status_msg.left_cam_connected = (
            self.left_cam_connect_if.check_connection()
            if self.left_cam_connect_if is not None else False)
        status_msg.right_cam_connected = (
            self.right_cam_connect_if.check_connection()
            if self.right_cam_connect_if is not None else False)
        status_msg.left_cam_topic = left_ns
        status_msg.right_cam_topic = right_ns
        status_msg.left_image_topic = left_img
        status_msg.right_image_topic = right_img

        # Depth map topic published by our IDX interface
        status_msg.depth_map_topic = nepi_sdk.create_namespace(
            self.node_namespace, IDX_DEPTH_SUBTOPIC)

        # Framerate + depth summary
        status_msg.max_framerate = self.max_framerate
        status_msg.valid_ratio = float(self.stereo_data_dict.get('valid_ratio', 0.0))
        status_msg.result_min_depth_mm = float(self.stereo_data_dict.get('result_min_depth_mm', 0.0))
        status_msg.result_max_depth_mm = float(self.stereo_data_dict.get('result_max_depth_mm', 0.0))

        if self.node_if is not None:
            self.node_if.publish_pub('status_pub', status_msg)


    #######################
    # Utility Functions
    #######################

    def cleanup_actions(self):
        self.msg_if.pub_info("CUSTOM_STEREO: Shutting down: Executing script cleanup actions")
        if self.left_cam_connect_if is not None:
            self.left_cam_connect_if.unregister()
        if self.right_cam_connect_if is not None:
            self.right_cam_connect_if.unregister()
        # TODO: release any per-frame camera subscriptions created by the wiring.


#########################################
# Main
#########################################
if __name__ == '__main__':
    NepiCustomStereoApp()
