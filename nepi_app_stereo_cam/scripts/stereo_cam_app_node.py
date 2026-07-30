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

"""Stereo Depth app node.

Assembled from the NEPI app template plus the developer-authored
idx_stereo_cam_node.py logic. It brings up:

  * TWO ConnectIDXDeviceIF selectors (left + right camera), each owning its own
    connect namespace (<node>/left_cam_connect and <node>/right_cam_connect),
    following nepi_app_idx_connect. The RUI drives selection through the
    Nepi_IF_ConnectIDX component.
  * A "processes" dropdown (available_processes / selected_process +
    set_selected_process / reload_processes), sourced from stereo_settings
    (create/update_processes_dict), following nepi_app_pan_tilt_auto.
  * A DepthMapIF that publishes the depth_map data product straight out of the
    depth loop: raw 32FC1 millimeter depth on <node>/depth_map, plus the
    colorized <node>/depth_map/depth_map_image the RUI viewer renders, plus the
    matching save-data channel. This app is a depth PRODUCER, not a camera, so
    it does not register as a virtual IDX device (which would also self-list it
    in its own left/right camera dropdowns).
  * A stereo calibration panel (RUI NepiAppCustomStereo-Calibration.js ->
    the set_calib_* / capture_calib_frame / solve_calib / clear_calib /
    load_calib topics), backed by calibrate.StereoCalibrator. Solving writes
    the .npz, loads it into a Rectifier, and pushes the measured
    focal_length_px / baseline_mm into every process so depth is true mm.

THE PER-FRAME PATH: updaterCb keeps image subscriptions pointed at whichever
two cameras the RUI selectors have chosen; the image callbacks decode each
sensor_msgs/Image into a cv2 BGR frame; grabPair() hands back the nearest
time-matched L/R pair. Depth AND calibration capture both read their frames
through grabPair(), so both run off the same synchronized pair.
"""

import os
import time
import copy
import importlib
import threading
from collections import deque

import numpy as np
import cv2

from std_msgs.msg import String, Empty, Float32
from sensor_msgs.msg import Image

from nepi_interfaces.msg import UpdateFloat

from nepi_app_stereo_cam.msg import NepiAppStereoCamStatus

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_img

from nepi_api.node_if import NodeClassIF
from nepi_api.messages_if import MsgIF

# NEPI data + IDX connect interfaces
from nepi_api.data_if import DepthMapIF
from nepi_api.connect_device_if_idx import ConnectIDXDeviceIF

# App-local sibling modules (co-located in scripts/). Bare imports -- the
# developer's original node used package-qualified names for the wrong package
# (nepi_idx_stereo_cam.*); reconciled here to bare sibling imports, matching
# stereo_settings.py / calibrate.py.
import calibrate
from calibrate import Rectifier, StereoCalibrator
import stereo_settings


#########################################
# Control Values
STATUS_PUBLISH_RATE_HZ = 1.0
UPDATE_RATE_HZ = 1.0

# Depth compute loop rate. The real ceiling is max_framerate, throttled inside
# updateDepthMap; this only has to tick faster than that so the throttle -- not
# the timer -- is what sets the rate. The timer is re-armed after each pass, so
# a compute slower than the tick just runs back-to-back rather than piling up.
DEPTH_RATE_HZ = 30.0

# Left and right are independent IDX devices, so their frames never arrive in
# lockstep. A pair further apart in capture time than this is not the same
# instant, and block matching it would put confidently wrong depth on anything
# moving. 100 ms is about one frame period at the default framerate cap.
FRAME_SYNC_TOLERANCE_S = 0.1
# Right-camera frames kept for pairing. Deep enough to absorb a second of
# arrival jitter at the default cap without growing without bound.
FRAME_BUFFER_LEN = 10

# IDX data product subtopics.
#
# The color subtopic is joined onto a SELECTED camera's namespace, which for an
# IDX connect is already the device's '<device>/idx' namespace (ConnectIDXDeviceIF
# get_selected_topic() -> selected_topic, and the connect subscribes to
# selected_topic + '/status' for DeviceIDXStatus, published at <device>/idx/status).
# So the color image resolves to <device>/idx/color_image -- the subtopic here must
# NOT re-include 'idx' or the topic doubles up to <device>/idx/idx/color_image.
IDX_COLOR_SUBTOPIC = 'color_image'
# Depth subtopics, joined onto THIS app node's namespace (not a selected device).
# DepthMapIF publishes the raw 32FC1 depth map at <node>/depth_map, and its
# DepthMapImageIF publishes the colorized version one level down, under that
# IF's own data product name -- so the RUI image viewer wants the '/image' one
# (the same rule NepiDeviceIDX.findImageTopic applies: for the depth_map product
# it skips the topic actually named 'depth_map', which is the float array).
DEPTH_SUBTOPIC = 'depth_map'
DEPTH_IMAGE_SUBTOPIC = 'depth_map/depth_map_image'

# Field of view reported alongside the depth map when there is no calibration to
# derive it from. Only a fallback: once rectifying, the real FOV comes out of the
# rectified focal length (see computeFovDeg).
FALLBACK_WIDTH_DEG = 100.0
FALLBACK_HEIGHT_DEG = 70.0

#########################################
# Node Class
#########################################

class NepiStereoCamApp(object):

    DEFAULT_NODE_NAME = "app_stereo_cam"
    DEFAULT_MAX_FRAMERATE = 10.0

    node_if = None
    depth_map_if = None
    left_cam_connect_if = None
    right_cam_connect_if = None
    msg_if = None

    # Processes (nepi stab/auto "processes" pattern; see stereo_settings.py)
    available_processes = list(stereo_settings.PROCESSES_DICT.keys())
    selected_process = stereo_settings.DEFAULT_PROCESS
    processes_dict = stereo_settings.create_processes_dict()
    process_ready = True

    # Rectification + calibration (driven from the RUI; see calibrate.py)
    rectifier = None
    calibrator = None
    calib_file = None
    calib_message = 'no calibration loaded'
    calib_epipolar_rms_px = 0.0

    # Framerate throttle + runtime data
    max_framerate = DEFAULT_MAX_FRAMERATE
    dm_data_last_time = None
    stereo_data_dict = stereo_settings.get_blank_data_dict()

    # NOTE: no cached depth map. Each pass publishes the array it just computed
    # and lets go of it -- the colorizer inside DepthMapIF overwrites the array
    # it is handed (nan / out-of-range pixels), so a kept reference would go
    # stale-and-wrong rather than merely stale. DepthMapIF publishes its own
    # rate / publishing status at <node>/depth_map/status.

    # Per-frame camera wiring. left_frame is the newest left frame; the right
    # side keeps a short history so grabPair() can pick the nearest time match
    # rather than whatever happened to arrive last.
    frame_lock = None
    left_frame = None
    right_frames = None
    left_sub = None
    right_sub = None
    left_img_topic = 'None'
    right_img_topic = 'None'
    last_pair_dt_s = 0.0

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

        # ---- Per-frame camera wiring ----
        # The image callbacks run on ROS subscriber threads while grabPair() runs
        # on the depth timer thread, so the frame stores are lock-guarded.
        self.frame_lock = threading.Lock()
        self.left_frame = None
        self.right_frames = deque(maxlen=FRAME_BUFFER_LEN)

        # ---- Calibration / rectification ----
        # Calibration is driven from the RUI "Stereo Calibration" panel (see the
        # calib_* subscribers below and calibrate.StereoCalibrator). The solved
        # .npz is loaded into a Rectifier, whose true focal_length_px /
        # baseline_mm are pushed into every process's settings so depth comes out
        # in real mm. A previously saved calibration is reloaded in initCb().
        self.rectifier = None
        self.calib_file = calibrate.default_calib_path()
        self.calibrator = StereoCalibrator()

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
            },
            # Depth compute rate cap. Owned by this node (it used to ride in on
            # the IDX device interface's setMaxFramerate hook).
            'max_framerate': {
                'namespace': self.node_namespace,
                'factory_val': self.DEFAULT_MAX_FRAMERATE
            },
            # Calibration is persisted as (file path + board description) only --
            # the maps themselves live in the .npz, which is reloaded on boot.
            'calib_file': {
                'namespace': self.node_namespace,
                'factory_val': self.calib_file
            },
            'calib_board_cols': {
                'namespace': self.node_namespace,
                'factory_val': calibrate.DEFAULT_BOARD_COLS
            },
            'calib_board_rows': {
                'namespace': self.node_namespace,
                'factory_val': calibrate.DEFAULT_BOARD_ROWS
            },
            'calib_square_mm': {
                'namespace': self.node_namespace,
                'factory_val': calibrate.DEFAULT_SQUARE_MM
            }
        }

        # Publishers Config Dict ####################
        self.PUBS_DICT = {
            'status_pub': {
                'namespace': self.node_namespace,
                'topic': 'status',
                'msg': NepiAppStereoCamStatus,
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
            },
            'set_max_framerate': {
                'namespace': self.node_namespace,
                'topic': 'set_max_framerate',
                'msg': Float32,
                'qsize': 10,
                'callback': self.setMaxFramerateCb,
                'callback_args': ()
            },
            # ---- RUI stereo calibration panel ----
            # Board description, one value at a time (UpdateFloat name/value):
            # 'board_cols', 'board_rows' or 'square_mm'.
            'set_calib_board_value': {
                'namespace': self.node_namespace,
                'topic': 'set_calib_board_value',
                'msg': UpdateFloat,
                'qsize': 10,
                'callback': self.setCalibBoardValueCb,
                'callback_args': ()
            },
            'set_calib_file': {
                'namespace': self.node_namespace,
                'topic': 'set_calib_file',
                'msg': String,
                'qsize': 10,
                'callback': self.setCalibFileCb,
                'callback_args': ()
            },
            'capture_calib_frame': {
                'namespace': self.node_namespace,
                'topic': 'capture_calib_frame',
                'msg': Empty,
                'qsize': 10,
                'callback': self.captureCalibFrameCb,
                'callback_args': ()
            },
            'solve_calib': {
                'namespace': self.node_namespace,
                'topic': 'solve_calib',
                'msg': Empty,
                'qsize': 10,
                'callback': self.solveCalibCb,
                'callback_args': ()
            },
            'clear_calib': {
                'namespace': self.node_namespace,
                'topic': 'clear_calib',
                'msg': Empty,
                'qsize': 10,
                'callback': self.clearCalibCb,
                'callback_args': ()
            },
            'load_calib': {
                'namespace': self.node_namespace,
                'topic': 'load_calib',
                'msg': Empty,
                'qsize': 10,
                'callback': self.loadCalibCb,
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
        # Depth map output.
        #
        # DepthMapIF owns the whole depth_map data product: the raw 32FC1 pub at
        # <node>/depth_map, the colorized depth_map_image (pub_image=True) that
        # the RUI viewer renders along with its render/overlay controls, and the
        # save-data channel for the product. It is given its OWN node_if (not
        # this node's) because its pub keys are generic 'data_pub'/'status_pub'
        # names that would collide with ours on a shared node_if.
        self.depth_map_if = DepthMapIF(
            namespace=self.node_namespace,
            data_product='depth_map',
            # Same descriptions idx_zed_node reports for its depth_map: the depth
            # comes from a stereo pair and is referenced to the left lens.
            data_source_description='stereo_camera',
            data_ref_description='left_camera_lense',
            perspective='pov',
            pub_image=True,
            log_name_list=[self.node_name],
            msg_if=self.msg_if
        )
        self.depth_map_if.wait_for_ready()

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
        self.initCb(do_updates=True)

        time.sleep(1)
        nepi_sdk.start_timer_process(float(1) / UPDATE_RATE_HZ, self.updaterCb, oneshot=True)
        # Depth runs on its own timer: the slow updaterCb is for discovery and
        # re-wiring, and running depth off it would cap the output at 1 Hz no
        # matter what max_framerate says.
        nepi_sdk.start_timer_process(float(1) / DEPTH_RATE_HZ, self.depthCb, oneshot=True)
        nepi_sdk.start_timer_process(float(1) / STATUS_PUBLISH_RATE_HZ, self.statusPublishCb)

        time.sleep(1)
        self.msg_if.pub_info("Initialization Complete")

        nepi_sdk.on_shutdown(self.cleanup_actions)
        nepi_sdk.spin()


    ###################
    ## Processes settings back-and-forth with the RUI

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
        # Cap on the depth compute rate (enforced in updateDepthMap). Returns
        # (success, err_str).
        if max_framerate is None or max_framerate <= 0.0:
            return False, "max framerate must be > 0, got " + str(max_framerate)
        self.max_framerate = float(max_framerate)
        return True, ""

    def getFramerate(self):
        return self.max_framerate


    ###################
    ## Stereo calibration (RUI "Stereo Calibration" panel)
    #
    # Flow: point the board at both cameras -> Capture (repeat ~10-20x at varied
    # distances / angles / image corners) -> Solve. Solve writes the .npz to
    # calib_file, loads it into a Rectifier, and pushes the measured
    # focal_length_px / baseline_mm into every process so depth is real mm.
    # Every step sets self.calib_message, which the RUI displays verbatim.

    def applyRectifier(self, rectifier):
        """Install a Rectifier and push its geometry into every process."""
        self.rectifier = rectifier
        if rectifier is None:
            return
        for settings in self.processes_dict.values():
            settings['focal_length_px'] = rectifier.focal_length_px
            settings['baseline_mm'] = rectifier.baseline_mm
        if self.node_if is not None:
            self.node_if.set_param('processes_dict', self.processes_dict)

    def loadCalibration(self, calib_file=None, quiet=False):
        """Load a saved .npz and start rectifying. Returns (ok, message)."""
        calib_file = calib_file if calib_file is not None else self.calib_file
        if not calib_file or not os.path.isfile(calib_file):
            message = 'no calibration file at ' + str(calib_file)
            if not quiet:
                self.msg_if.pub_warn(message)
            self.calib_message = message
            return False, message
        try:
            rectifier = Rectifier(calib_file)
        except Exception as e:
            # A truncated / wrong-format .npz must not take the node down.
            message = 'failed to load ' + str(calib_file) + ': ' + str(e)
            self.msg_if.pub_warn(message)
            self.calib_message = message
            return False, message
        self.applyRectifier(rectifier)
        self.calib_file = calib_file
        message = ('loaded ' + os.path.basename(calib_file) +
                   ': f %.1f px, baseline %.1f mm, %dx%d' % (
                       rectifier.focal_length_px, rectifier.baseline_mm,
                       rectifier.image_size[0], rectifier.image_size[1]))
        self.msg_if.pub_info(message)
        self.calib_message = message
        return True, message

    def setCalibBoardValue(self, name, value):
        """RUI edited one of the board description values."""
        if name in ('board_cols', 'cols'):
            ok, message = self.calibrator.set_board(cols=int(round(value)))
        elif name in ('board_rows', 'rows'):
            ok, message = self.calibrator.set_board(rows=int(round(value)))
        elif name in ('square_mm', 'calib_square_mm'):
            ok, message = self.calibrator.set_board(square_mm=float(value))
        else:
            return False, 'unknown calibration board value: ' + str(name)
        if ok and self.node_if is not None:
            self.node_if.set_param('calib_board_cols', self.calibrator.cols)
            self.node_if.set_param('calib_board_rows', self.calibrator.rows)
            self.node_if.set_param('calib_square_mm', self.calibrator.square_mm)
            self.node_if.save_config()
        return ok, message

    def captureCalibFrame(self):
        """Grab the live L/R pair and look for the board in both."""
        ok, left, right, _ = self.grabPair()
        if not ok:
            return False, ('no synchronized camera frames -- select both cameras '
                           'and check they are publishing')
        # Raw (unrectified) frames on purpose: calibration is what PRODUCES the
        # rectification, so feeding it rectified frames would bake the current
        # calibration into the new one.
        return self.calibrator.capture(left, right)

    def solveCalibration(self):
        """Solve from the captured views, save, and start rectifying."""
        ok, message, info = self.calibrator.solve(self.calib_file)
        if not ok:
            return False, message
        loaded, load_message = self.loadCalibration(self.calib_file)
        if not loaded:
            return False, message + ' | ' + load_message
        self.calib_epipolar_rms_px = float(info['epipolar_rms_px'])
        if self.node_if is not None:
            self.node_if.set_param('calib_file', self.calib_file)
            self.node_if.save_config()
        # Keep the captures: a poor epipolar RMS is usually fixed by adding a
        # few more views and solving again, not by starting over.
        return True, message


    ###################
    ## Frame acquisition + depth map (developer's idx_custom_stereo_node logic)

    def grabPair(self):
        """Return (ok, left_bgr, right_bgr, timestamp) for the closest time match.

        The two cameras are independent IDX devices, so their frames do not
        arrive in lockstep: the newest left frame is matched against a short
        history of right frames and the nearest one wins. A pair further apart
        than FRAME_SYNC_TOLERANCE_S is refused rather than returned -- matching
        a stale frame against a fresh one yields confidently wrong depth on
        anything that moved in between.
        """
        with self.frame_lock:
            left = self.left_frame
            rights = list(self.right_frames)
        if left is None or len(rights) == 0:
            return False, None, None, None

        best = min(rights, key=lambda right: abs(left['ts'] - right['ts']))
        best_dt = abs(left['ts'] - best['ts'])
        self.last_pair_dt_s = best_dt
        if best_dt > FRAME_SYNC_TOLERANCE_S:
            return False, None, None, None
        return True, left['img'], best['img'], left['ts']

    def updateDepthMap(self):
        # Compute a depth map and publish it through DepthMapIF. Returns True if
        # a new depth map was published.
        #
        # Skip the (expensive) block matching while nothing needs the data.
        # DepthMapIF.needs_data_check() is True when anything wants the product:
        # a raw depth_map subscriber, a colorized depth_map_image subscriber (the
        # RUI viewer), or an enabled save / snapshot. The camera image
        # subscriptions stay up regardless -- the calibration panel reads its
        # frames through the same grabPair() path and must keep working with
        # depth paused.
        if self.depth_map_if is None or self.depth_map_if.needs_data_check() is False:
            return False

        # A reload swaps the process registry out from under us.
        if self.process_ready is False:
            return False

        # framerate throttle
        last_time = self.dm_data_last_time
        current_time = nepi_utils.get_time()
        if last_time is not None:
            fr_delay = float(1) / self.max_framerate
            if (current_time - last_time) < fr_delay:
                return False

        # Grab a synchronized L/R pair from the two selected cameras.
        ok, left, right, timestamp = self.grabPair()
        if not ok:
            return False

        # Rectify (compute_depth_map assumes rectified input). With no
        # calibration there is nothing to rectify with, and block matching raw
        # frames against placeholder geometry yields numbers that look like depth
        # but are not -- same call as the resolution-mismatch refusal below. The
        # calibration panel is unaffected: it reads grabPair() directly, so the
        # capture/solve flow still works with depth held off. calib_message is
        # deliberately left alone here so this does not stomp on the panel's
        # per-press feedback.
        if self.rectifier is None:
            self.msg_if.pub_warn(
                "No calibration loaded -- depth needs rectified frames",
                throttle_s=10.0)
            return False

        if not self.rectifier.matches(left):
            # Camera resolution changed since calibration -- the maps no
            # longer apply, and matching unrectified frames would produce
            # confidently wrong depth. Refuse rather than publish garbage.
            message = (
                'frame %dx%d does not match calibration %dx%d -- recalibrate'
                % (left.shape[1], left.shape[0],
                   self.rectifier.image_size[0], self.rectifier.image_size[1]))
            # Warn once per distinct problem instead of every update tick.
            if message != self.calib_message:
                self.msg_if.pub_warn(message)
                self.calib_message = message
            return False
        left, right = self.rectifier.rectify(left, right)

        # Run the selected stereo process (fills self.stereo_data_dict). A
        # reload_processes between the check above and here would leave the two
        # dicts briefly out of step, so read both defensively.
        process = stereo_settings.PROCESSES_DICT.get(self.selected_process, None)
        settings = self.processes_dict.get(self.selected_process, None)
        if process is None or settings is None:
            return False
        self.stereo_data_dict, _ = process['process_function'](
            left, right, self.stereo_data_dict, settings)

        np_depth_map = self.stereo_data_dict['depth_map']   # (H,W) float32 mm
        # Match the ZED stereo convention: invalid pixels -> nan instead of 0.0.
        np_depth_map[np_depth_map <= 0.0] = np.nan

        # NEPI depth_map data products carry MILLIMETER values while their range
        # bounds are given in METERS -- the colorizer scales the bounds by 1e3
        # (nepi_img.npDepthMap_to_cv2ColorImg), which is why idx_zed_node converts
        # its meter measure up to mm. compute_depth_map already returns mm, so
        # only the bounds need converting here. Feeding meters instead would put
        # every pixel below min_range_m*1e3 and render a flat single color.
        controls = settings.get('stereo_controls_dict', {})
        min_range_m = float(controls.get('min_depth_mm', 0.0)) / 1000.0
        max_range_m = float(controls.get('max_depth_mm', 1000.0)) / 1000.0
        width_deg, height_deg = self.computeFovDeg(np_depth_map)

        # Publish the raw depth map (32FC1), the colorized depth_map_image and the
        # save-data copy in one call. Stamped with the CAPTURE time of the pair it
        # came from, not the time the compute finished, so consumers can line it
        # up against the source images.
        #
        # The array is handed off, not kept: the colorizer inside overwrites nan /
        # out-of-range pixels in place, so np_depth_map must not be read after
        # this. Each pass gets a fresh array from the process function.
        self.depth_map_if.publish_np_depth_map(
            np_depth_map,
            encoding='32FC1',
            width_deg=width_deg,
            height_deg=height_deg,
            min_range_m=min_range_m,
            max_range_m=max_range_m,
            timestamp=timestamp)

        self.dm_data_last_time = nepi_utils.get_time()
        return True

    def computeFovDeg(self, np_depth_map):
        # Field of view the depth map covers, reported alongside it so viewers can
        # convert a pixel to a bearing. Depth only runs on rectified frames, so the
        # rectified focal length is available whenever there is a map to publish:
        # fov = 2*atan(size_px / (2*f)). One focal length is used for both axes --
        # stereoRectify's P1 has fy == fx, so pixels are square by construction.
        if self.rectifier is None or self.rectifier.focal_length_px <= 0.0:
            return FALLBACK_WIDTH_DEG, FALLBACK_HEIGHT_DEG
        height, width = np_depth_map.shape[0:2]
        focal_px = float(self.rectifier.focal_length_px)
        width_deg = 2.0 * np.degrees(np.arctan(float(width) / (2.0 * focal_px)))
        height_deg = 2.0 * np.degrees(np.arctan(float(height) / (2.0 * focal_px)))
        return float(width_deg), float(height_deg)


    ###################
    ## Per-frame camera subscriptions

    def updateImageSubs(self):
        # The RUI's left/right selectors drive the two ConnectIDXDeviceIF
        # instances; whenever either resolves to a different color image topic,
        # move our image subscription with it.
        _, left_img, _, right_img = self.computeImageTopics()
        if left_img != self.left_img_topic:
            self.subscribeLeft(left_img)
        if right_img != self.right_img_topic:
            self.subscribeRight(right_img)

    def subscribeLeft(self, topic):
        self.left_sub = self.unsubscribeImage(self.left_sub)
        with self.frame_lock:
            self.left_frame = None
        self.left_img_topic = topic
        if topic not in (None, 'None', ''):
            # qsize 1: on a depth map that takes longer than a frame period,
            # queued frames are stale by the time they are read -- drop them and
            # work from the newest instead.
            self.left_sub = nepi_sdk.create_subscriber(
                topic, Image, self.leftImageCb, queue_size=1)
            self.msg_if.pub_info("Subscribed to left camera image: " + str(topic))

    def subscribeRight(self, topic):
        self.right_sub = self.unsubscribeImage(self.right_sub)
        with self.frame_lock:
            self.right_frames.clear()
        self.right_img_topic = topic
        if topic not in (None, 'None', ''):
            self.right_sub = nepi_sdk.create_subscriber(
                topic, Image, self.rightImageCb, queue_size=1)
            self.msg_if.pub_info("Subscribed to right camera image: " + str(topic))

    def unsubscribeImage(self, sub):
        # Returns None so callers can assign the result straight back.
        if sub is not None:
            try:
                sub.unregister()
            except Exception as e:
                self.msg_if.pub_warn("Failed to unregister image subscriber: " + str(e))
        return None

    def leftImageCb(self, msg):
        frame = self.rosImgToCv2(msg)
        if frame is None:
            return
        with self.frame_lock:
            self.left_frame = {'img': frame, 'ts': self.frameTimestamp(msg)}

    def rightImageCb(self, msg):
        frame = self.rosImgToCv2(msg)
        if frame is None:
            return
        with self.frame_lock:
            self.right_frames.append({'img': frame, 'ts': self.frameTimestamp(msg)})

    def rosImgToCv2(self, msg):
        # bgr8 rather than passthrough: compute_depth_map and the chessboard
        # finder both branch on channel count, so the two cameras must not
        # disagree about it (one mono, one color would).
        try:
            return nepi_img.rosimg_to_cv2img(msg, encoding='bgr8')
        except Exception as e:
            self.msg_if.pub_warn("Failed to convert camera image: " + str(e),
                                 throttle_s=5.0)
            return None

    def frameTimestamp(self, msg):
        # Capture time is what pairing needs. Some publishers leave the header
        # stamp at zero, in which case arrival time is the only ordering
        # available -- worse, but still monotonic and shared by both cameras.
        timestamp = nepi_sdk.sec_from_msg_stamp(msg.header.stamp)
        if timestamp <= 0.0:
            timestamp = nepi_utils.get_time()
        return timestamp


    ###################
    ## App Callbacks

    def depthCb(self, timer):
        # Depth runs on its own timer so block matching neither blocks the image
        # subscriber threads nor gets capped by the slow updaterCb rate.
        try:
            self.updateDepthMap()
        except Exception as e:
            self.msg_if.pub_warn("Depth update failed: " + str(e), throttle_s=5.0)
        nepi_sdk.start_timer_process(float(1) / DEPTH_RATE_HZ, self.depthCb, oneshot=True)

    def updaterCb(self, timer):
        # Keep the per-frame image subscriptions pointed at whichever cameras the
        # left/right selectors currently have chosen.
        self.updateImageSubs()

        nepi_sdk.start_timer_process(float(1) / UPDATE_RATE_HZ, self.updaterCb, oneshot=True)

    def setSelectedProcessCb(self, msg):
        self.msg_if.pub_info(str(msg))
        process_name = msg.data
        if self.setSelectedProcess(process_name):
            self.publish_status()
            if self.node_if is not None:
                self.node_if.set_param('selected_process', self.selected_process)
                self.node_if.save_config()

    def setMaxFramerateCb(self, msg):
        ok, message = self.setMaxFramerate(msg.data)
        if ok:
            if self.node_if is not None:
                self.node_if.set_param('max_framerate', self.max_framerate)
                self.node_if.save_config()
        else:
            self.msg_if.pub_warn(message)
        self.publish_status()

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

    ## RUI stereo calibration panel callbacks. Each one records the result in
    ## self.calib_message and republishes status, so the panel always shows what
    ## the last press actually did.

    def _finishCalibAction(self, ok, message):
        self.calib_message = message
        if ok:
            self.msg_if.pub_info(message)
        else:
            self.msg_if.pub_warn(message)
        self.publish_status()

    def setCalibBoardValueCb(self, msg):
        ok, message = self.setCalibBoardValue(msg.name, msg.value)
        self._finishCalibAction(ok, message)

    def setCalibFileCb(self, msg):
        calib_file = msg.data.strip()
        if not calib_file:
            self._finishCalibAction(False, 'calibration file path cannot be empty')
            return
        if not calib_file.endswith('.npz'):
            calib_file = calib_file + '.npz'
        self.calib_file = calib_file
        if self.node_if is not None:
            self.node_if.set_param('calib_file', self.calib_file)
            self.node_if.save_config()
        # Loading is best-effort: the path may be where a future solve will
        # WRITE, in which case there is nothing to read yet.
        ok, message = self.loadCalibration(self.calib_file, quiet=True)
        if not ok:
            message = 'calibration file set to ' + self.calib_file + ' (not present yet)'
        self._finishCalibAction(True, message)

    def captureCalibFrameCb(self, msg):
        ok, message = self.captureCalibFrame()
        self._finishCalibAction(ok, message)

    def solveCalibCb(self, msg):
        ok, message = self.solveCalibration()
        self._finishCalibAction(ok, message)

    def clearCalibCb(self, msg):
        ok, message = self.calibrator.clear()
        self._finishCalibAction(ok, message)

    def loadCalibCb(self, msg):
        ok, message = self.loadCalibration(self.calib_file)
        self._finishCalibAction(ok, message)

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
            # A reload re-seeds settings from the module defaults, which would
            # otherwise throw away the calibrated geometry.
            self.applyRectifier(self.rectifier)
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
            self.setMaxFramerate(self.node_if.get_param('max_framerate'))
            # Restore the calibration the operator saved from the RUI: the board
            # description plus the .npz path, which is reloaded if it still
            # exists (quiet -- a device with no calibration yet is normal).
            self.calib_file = self.node_if.get_param('calib_file')
            self.calibrator.set_board(
                cols=self.node_if.get_param('calib_board_cols'),
                rows=self.node_if.get_param('calib_board_rows'),
                square_mm=self.node_if.get_param('calib_square_mm'))
            self.loadCalibration(self.calib_file, quiet=True)
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
        # right_img); 'None' where unselected.
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
        status_msg = NepiAppStereoCamStatus()

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

        # Depth map topics published by our DepthMapIF: the raw 32FC1 map, and the
        # colorized image the RUI viewer renders.
        status_msg.depth_map_topic = nepi_sdk.create_namespace(
            self.node_namespace, DEPTH_SUBTOPIC)
        status_msg.depth_map_image_topic = nepi_sdk.create_namespace(
            self.node_namespace, DEPTH_IMAGE_SUBTOPIC)

        # Stereo calibration panel state
        status_msg.calib_loaded = (self.rectifier is not None)
        status_msg.calib_file = str(self.calib_file)
        status_msg.calib_board_cols = int(self.calibrator.cols)
        status_msg.calib_board_rows = int(self.calibrator.rows)
        status_msg.calib_square_mm = float(self.calibrator.square_mm)
        status_msg.calib_capture_count = int(self.calibrator.count)
        status_msg.calib_min_captures = int(calibrate.MIN_PAIRS)
        status_msg.calib_message = str(self.calib_message)
        status_msg.calib_focal_length_px = (
            float(self.rectifier.focal_length_px) if self.rectifier is not None else 0.0)
        status_msg.calib_baseline_mm = (
            float(self.rectifier.baseline_mm) if self.rectifier is not None else 0.0)
        status_msg.calib_epipolar_rms_px = float(self.calib_epipolar_rms_px)

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
        if self.depth_map_if is not None:
            self.depth_map_if.unregister()
        if self.left_cam_connect_if is not None:
            self.left_cam_connect_if.unregister()
        if self.right_cam_connect_if is not None:
            self.right_cam_connect_if.unregister()
        # Release the per-frame camera subscriptions.
        self.subscribeLeft('None')
        self.subscribeRight('None')


#########################################
# Main
#########################################
if __name__ == '__main__':
    NepiStereoCamApp()
