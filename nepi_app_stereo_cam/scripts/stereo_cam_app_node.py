#!/usr/bin/env python
#
# Copyright (c) 2026 Numurus <https://www.numurus.com>.
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
    PROCESSES_DICT, following nepi_app_pan_tilt_auto. Each process's tunable
    values are a nepi_controls control set owned by its OWN ControlsIF, one per
    process, keyed by process name -- see setupProcessControlsIFs(). The
    selector itself is deliberately NOT a control; see the comment there.
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
sensor_msgs/Image into a cv2 BGR frame and buffers it. Depth reads grabPair()
(newest left, nearest right -- latency matters); calibration capture reads
grabCalibPair() (best-matched pair anywhere in the buffers, plus a measurement of
whether the scene was moving -- accuracy matters and age is free).
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
from nepi_api.system_if import ControlsIF

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
# Calibration holds the L/R pair to a much tighter standard than depth does.
# Depth tolerates 100 ms because a stale pixel is a local error in one frame;
# calibration does not, because the corner positions in that pair become
# PERMANENT constraints in the solve. A board being tilted by hand moves several
# pixels in 100 ms, and that motion enters stereoCalibrate as an apparent
# disagreement between the two cameras -- inflating the stereo RMS and the
# epipolar error while each camera on its own still looks fine. A pair over this
# gap is REFUSED, not warned about, for the same reason a moving scene is: it
# reads as an ordinary success and silently poisons the solve.
# Only enforced when both frames carry real header stamps -- see captureCalibFrame.
CALIB_SYNC_MAX_S = 0.02
# Scene motion (mean abs 8-bit frame-to-frame difference) above which a
# calibration capture is REFUSED rather than warned about. This is the check that
# does not depend on the cameras being synchronized, or on their timestamps being
# real -- if nothing moved between consecutive frames of either camera, the L/R
# pair is simultaneous for calibration purposes however far apart it is stamped.
# Set well above sensor noise (~1) so a still scene is never rejected; a board
# being repositioned by hand sits far above it. Raise it if a noisy sensor or
# auto-exposure hunting makes a genuinely stationary board fail.
CALIB_MOTION_MAX = 3.0

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

# Controls name of the example control set: the <app>/example_controls namespace one
# ControlsIF owns and the Nepi_IF_Controls box at the bottom of this app's RUI column
# binds to. It belongs to no process and drives nothing here -- see
# setupExampleControlsIF().
#
# It is one instance among several. Every process in stereo_settings.PROCESSES_DICT
# owns a ControlsIF of its own, named after the process, because only one process is
# active at a time and the operator should be shown just that one's controls. The
# example set is the exception: always mounted, since there is no active/inactive
# question about it.
EXAMPLE_CONTROLS_NAME = "example_controls"

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

    # One ControlsIF per entry in stereo_settings.PROCESSES_DICT, keyed by process
    # name. There is no app-level process ControlsIF -- see the selector comment on
    # setupProcessControlsIFs().
    process_controls_ifs = None

    # The example control set copied from nepi_app_controls_sandbox. Not a process
    # control set and not keyed by process. See setupExampleControlsIF().
    example_controls_if = None

    # Processes (nepi stab/auto "processes" pattern; see stereo_settings.py)
    available_processes = list(stereo_settings.PROCESSES_DICT.keys())
    selected_process = stereo_settings.DEFAULT_PROCESS
    process_ready = True

    # Set once the first time a depth pass had to be skipped because a ControlsIF
    # reported a None controls dict. See getControlsValues() for what puts an IF in
    # that state; logged once rather than at the depth rate.
    logged_controls_dict_none = False

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

    # Per-frame camera wiring. BOTH sides keep a short history. Depth wants the
    # newest left frame and the right frame nearest it in time (latency matters).
    # Calibration wants the best-matched pair anywhere in the two buffers and does
    # not care how old it is, because a capture is used once, offline, and its
    # corner positions become permanent constraints in the solve -- see
    # grabPair() and grabCalibPair().
    frame_lock = None
    left_frames = None
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
        self.process_controls_ifs = dict()
        self.stereo_data_dict = stereo_settings.get_blank_data_dict()
        self.max_framerate = self.DEFAULT_MAX_FRAMERATE

        # ---- Per-frame camera wiring ----
        # The image callbacks run on ROS subscriber threads while grabPair() runs
        # on the depth timer thread, so the frame stores are lock-guarded.
        self.frame_lock = threading.Lock()
        self.left_frames = deque(maxlen=FRAME_BUFFER_LEN)
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
            # No 'processes_dict' param. Each process's control set is persisted by
            # its own ControlsIF, under that IF's own '<process>_controls_dict'
            # param in its own namespace.
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
            # No 'set_process_control_value'. Every process setting is a
            # nepi_controls control now, so an edit goes to the typed
            # set_<type>_control_value topic of that process's own ControlsIF
            # rather than arriving here as an UpdateFloat to be cast back.
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
        # Controls. Built after the app's NodeClassIF and before initCb, so the
        # first status publish can already report every controls namespace -- and
        # so initCb's loadCalibration() has ControlsIF instances to push the
        # measured focal_length_px / baseline_mm into.
        self.setupControlsIFs()

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
    ## Process selector
    #
    # There is no settings read/write path here any more. Each process's tunable
    # values live in that process's own ControlsIF, which owns the typed
    # set_<type>_control_value topics the RUI publishes to, the bounds/options
    # validation, and the param persistence. The node only reads a flat snapshot
    # once per depth pass -- see getControlsValues().

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
        self.applyCalibGeometry()

    # Write the measured focal_length_px / baseline_mm into every process's
    # ControlsIF, so depth comes out in real mm rather than off the placeholder
    # geometry the control defaults carry.
    #
    # ControlsIF.set_control_value() persists the change to that IF's own param, so
    # there is no set_param call here -- and no 'processes_dict' param left to write
    # it to. A no-op before setupProcessControlsIFs() has run: NodeClassIF fires
    # initCb (which reloads the calibration) while it is still being constructed.
    def applyCalibGeometry(self):
        if self.rectifier is None or self.process_controls_ifs is None:
            return
        for process_name in self.process_controls_ifs.keys():
            controls_if = self.process_controls_ifs[process_name]
            if controls_if is None:
                continue
            controls_if.set_control_value('focal_length_px',
                                          float(self.rectifier.focal_length_px))
            controls_if.set_control_value('baseline_mm',
                                          float(self.rectifier.baseline_mm))

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
        """Grab the best-synchronized L/R pair and look for the board in both."""
        ok, left, right, report = self.grabCalibPair()
        if not ok:
            return False, report

        # Refuse a moving scene outright rather than capturing it with a warning.
        # A mid-motion pair is not a marginal capture, it is a WRONG one: the two
        # cameras saw the board in different places, and the solve cannot tell
        # that apart from the cameras being further apart than they really are.
        # It looks like an ordinary success and quietly ruins the result, so the
        # only safe treatment is to not keep it.
        if report['motion'] > CALIB_MOTION_MAX:
            return False, ('scene is moving (%.1f, limit %.1f) -- let the board '
                           'come to rest, then hold it still for a second before '
                           'capturing; kept %d'
                           % (report['motion'], CALIB_MOTION_MAX,
                              self.calibrator.count))

        # Same reasoning as the motion check, on the other measurement. Refused
        # rather than warned about because a captured pair is never revisited:
        # once its corners are in the solve, nothing downstream can tell them
        # apart from good ones.
        #
        # Gated on 'stamped' deliberately. Without header stamps dt_s is the gap
        # between ARRIVAL times, which for two free-running USB cameras routinely
        # exceeds 20 ms on pairs that were in fact captured together -- enforcing
        # it there would refuse every capture and make calibration impossible on
        # exactly the cameras that need it most. Unstamped setups are covered by
        # the motion check above, which needs no timestamps to be meaningful.
        if report['stamped'] and report['dt_s'] > CALIB_SYNC_MAX_S:
            return False, ('L/R capture gap %.0f ms (limit %.0f ms) -- the two '
                           'cameras did not see the same instant; hold the board '
                           'still and capture again; kept %d'
                           % (report['dt_s'] * 1000.0, CALIB_SYNC_MAX_S * 1000.0,
                              self.calibrator.count))

        # Raw (unrectified) frames on purpose: calibration is what PRODUCES the
        # rectification, so feeding it rectified frames would bake the current
        # calibration into the new one.
        ok, message = self.calibrator.capture(left, right)
        if ok:
            message += ' [L/R %.0f ms, motion %.1f]' % (report['dt_s'] * 1000.0,
                                                        report['motion'])
            if not report['stamped']:
                # Say this rather than let a reassuring millisecond figure stand
                # in for a measurement the cameras never actually provided. Also
                # the only case that reaches here with a gap over the limit --
                # a stamped pair that far apart was refused above.
                message += (' (no header timestamps -- L/R gap is arrival time, '
                            'not capture time)')
        return ok, message

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
            lefts = list(self.left_frames)
            rights = list(self.right_frames)
        if len(lefts) == 0 or len(rights) == 0:
            return False, None, None, None

        left = lefts[-1]
        best = min(rights, key=lambda right: abs(left['ts'] - right['ts']))
        best_dt = abs(left['ts'] - best['ts'])
        self.last_pair_dt_s = best_dt
        if best_dt > FRAME_SYNC_TOLERANCE_S:
            return False, None, None, None
        return True, left['img'], best['img'], left['ts']

    def grabCalibPair(self):
        """Pick the best-synchronized L/R pair available, and say how good it is.

        Returns (ok, left_bgr, right_bgr, report_dict_or_reason).

        Different job from grabPair(), so a different rule. Depth needs the NEWEST
        pair and tolerates a loose match, because latency is the point and one
        stale pixel is a local error. A calibration capture is used once, offline,
        and the corner positions in it become permanent constraints on the solve
        -- so age is free and mismatch is not. This searches BOTH buffers for the
        globally closest pair instead of anchoring on the newest left frame, which
        alone typically halves the gap: the right frame from the same instant as
        the newest left frame has often not arrived yet.

        The report also carries a direct measurement of whether the scene was
        MOVING, which is what actually matters. Two free-running USB cameras are
        not synchronized at all -- there is up to a frame period of real skew no
        matter what the timestamps say -- so a small time difference does not make
        a pair simultaneous. A motionless scene does, whatever the difference.
        """
        with self.frame_lock:
            lefts = list(self.left_frames)
            rights = list(self.right_frames)
        if len(lefts) == 0 or len(rights) == 0:
            return False, None, None, ('no camera frames -- select both cameras '
                                       'and check they are publishing')

        left, right = min(((l, r) for l in lefts for r in rights),
                          key=lambda pair: abs(pair[0]['ts'] - pair[1]['ts']))
        dt_s = abs(left['ts'] - right['ts'])
        self.last_pair_dt_s = dt_s
        return True, left['img'], right['img'], {
            'dt_s': dt_s,
            # Only meaningful when BOTH sides carry real capture stamps.
            'stamped': bool(left.get('stamped') and right.get('stamped')),
            'motion': max(self.frameMotion(lefts), self.frameMotion(rights)),
        }

    def frameMotion(self, frames):
        """Mean abs 8-bit difference between the two newest frames of one camera.

        A stand-in for "is the scene holding still", and the check that does the
        real work here: it needs no timestamps and no camera synchronization, so
        it stays valid exactly where the time difference stops being trustworthy.
        Sensor noise alone lands around 1; a board being moved by hand is well
        clear of it. Returns 0.0 when there is nothing to compare against, which
        cannot raise a false alarm.
        """
        if len(frames) < 2:
            return 0.0
        newest, previous = frames[-1]['img'], frames[-2]['img']
        if newest.shape != previous.shape:
            return 0.0
        return float(np.mean(cv2.absdiff(newest, previous)))

    def updateDepthMap(self):
        # Compute a depth map and publish it through DepthMapIF. Returns True if
        # a new depth map was published.
        #
        # Skip the (expensive) block matching while nothing needs the data.
        # DepthMapIF.needs_data_check() is True when anything wants the product:
        # a raw depth_map subscriber, a colorized depth_map_image subscriber (the
        # RUI viewer), or an enabled save / snapshot. The camera image
        # subscriptions stay up regardless -- the calibration panel reads its
        # frames through the grabCalibPair() path and must keep working with
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
        # calibration panel is unaffected: it reads grabCalibPair() directly, so the
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

        # Run the selected stereo process (fills self.stereo_data_dict). The
        # process function is handed a flat {control_name: value} snapshot read
        # off THIS process's ControlsIF, once per pass -- a control is read there
        # by exactly the key it is authored under in stereo_settings. A
        # reload_processes between the check above and here would leave the
        # registry and the IF map briefly out of step, so read both defensively.
        process = stereo_settings.PROCESSES_DICT.get(self.selected_process, None)
        controls_if = None
        if self.process_controls_ifs is not None:
            controls_if = self.process_controls_ifs.get(self.selected_process, None)
        if process is None or controls_if is None:
            return False
        process_controls_dict = self.getControlsValues(controls_if)
        if process_controls_dict is None:
            if self.logged_controls_dict_none is False:
                self.logged_controls_dict_none = True
                self.msg_if.pub_warn("Depth pass skipped: controls for '" +
                                     str(self.selected_process) +
                                     "' read back as None -- see getControlsValues()")
            return False
        self.stereo_data_dict, _ = process['process_function'](
            left, right, self.stereo_data_dict, process_controls_dict)

        np_depth_map = self.stereo_data_dict['depth_map']   # (H,W) float32 mm
        # Match the ZED stereo convention: invalid pixels -> nan instead of 0.0.
        np_depth_map[np_depth_map <= 0.0] = np.nan

        # NEPI depth_map data products carry MILLIMETER values while their range
        # bounds are given in METERS -- the colorizer scales the bounds by 1e3
        # (nepi_img.npDepthMap_to_cv2ColorImg), which is why idx_zed_node converts
        # its meter measure up to mm. compute_depth_map already returns mm, so
        # only the bounds need converting here. Feeding meters instead would put
        # every pixel below min_range_m*1e3 and render a flat single color.
        min_range_m = float(process_controls_dict.get('min_depth_mm', 0.0)) / 1000.0
        max_range_m = float(process_controls_dict.get('max_depth_mm', 1000.0)) / 1000.0
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
            self.left_frames.clear()
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
            self.left_frames.append(self.frameRecord(msg, frame))

    def rightImageCb(self, msg):
        frame = self.rosImgToCv2(msg)
        if frame is None:
            return
        with self.frame_lock:
            self.right_frames.append(self.frameRecord(msg, frame))

    def frameRecord(self, msg, frame):
        # 'stamped' travels with the frame because it decides whether a measured
        # L/R time difference means anything at all: a driver that leaves the
        # header stamp at zero leaves frameTimestamp() reporting ARRIVAL time,
        # which folds transport and scheduling jitter into the number and can
        # read as well-synchronized when the exposures were not.
        timestamp, stamped = self.frameTimestamp(msg)
        return {'img': frame, 'ts': timestamp, 'stamped': stamped}

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
        # Returns (timestamp, from_header) so callers can say how much the
        # resulting time difference is actually worth.
        timestamp = nepi_sdk.sec_from_msg_stamp(msg.header.stamp)
        if timestamp <= 0.0:
            return nepi_utils.get_time(), False
        return timestamp, True


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
        # Adds a missing .npz and puts a bare filename in the device cal folder
        # (next to the ZED cal files) rather than the node's working directory.
        self.calib_file = calibrate.resolve_calib_path(calib_file)
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

    # Reload the stereo_settings module so edits to the processes registry are
    # picked up without restarting the node (pattern from pan_tilt_auto
    # reloadAutosCb).
    #
    # A reload can ADD a process -- a new PROCESSES_DICT entry gets its own
    # ControlsIF in setupProcessControlsIFs(). It cannot change or remove the
    # controls of a process that already has one: releasing a ControlsIF means
    # ControlsIF.unregister(), whose first line calls a self.unsubscribe_topic()
    # that nepi_api/system_if.py never defines, so it raises AttributeError before
    # releasing anything. Editing an existing process's control set still needs a
    # node restart, and a process dropped from the module keeps its namespace until
    # then -- it just stops being offered in available_processes.
    def reloadProcessesCb(self, msg):
        self.process_ready = False
        self.publish_status()
        nepi_sdk.sleep(1)
        try:
            importlib.reload(stereo_settings)
            self.available_processes = list(stereo_settings.PROCESSES_DICT.keys())
            if len(self.available_processes) == 0:
                self.msg_if.pub_warn("Reloaded stereo_settings registers no processes")
            else:
                if self.selected_process not in self.available_processes:
                    self.selected_process = self.available_processes[0]
                self.setupProcessControlsIFs()
                # A newly added process starts from the module's control defaults,
                # which carry placeholder geometry; re-push the calibrated values.
                self.applyCalibGeometry()
                self.msg_if.pub_info("Stereo processes reloaded: " + str(self.available_processes))
            self.process_ready = True
        except Exception as e:
            self.msg_if.pub_warn("Failed to reload stereo_settings module: " + str(e))
            self.process_ready = True
        self.publish_status()


    #######################
    ### Config Functions

    def initCb(self, do_updates=False):
        if self.node_if is not None:
            # The selector is this app's param, so a restart restores the process
            # the operator last picked. A param naming a process the module no
            # longer registers falls back to the module default rather than
            # leaving the app pointed at nothing. Each process's control VALUES
            # are restored by that process's own ControlsIF from its own param.
            selected_process = self.node_if.get_param('selected_process')
            if selected_process in self.available_processes:
                self.selected_process = selected_process
            else:
                self.selected_process = stereo_settings.DEFAULT_PROCESS
            self.setMaxFramerate(self.node_if.get_param('max_framerate'))
            # Restore the calibration the operator saved from the RUI: the board
            # description plus the .npz path, which is reloaded if it still
            # exists (quiet -- a device with no calibration yet is normal).
            # Rebase a stored path that holds no calibration onto the current
            # default folder (the ZED cal folder). Configs written before
            # default_calib_folder() reliably resolved to that folder carry a home
            # fallback path, and the stale param would keep new solves out of the
            # cal folder forever. A stored path that DOES have a file is left
            # alone: that is a real calibration the operator chose.
            calib_file = calibrate.resolve_calib_path(self.node_if.get_param('calib_file'))
            if not os.path.isfile(calib_file):
                calib_file = os.path.join(calibrate.default_calib_folder(),
                                          os.path.basename(calib_file))
            self.calib_file = calib_file
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
        self.resetControlsIFs(factory=False)
        if do_updates:
            pass
        self.initCb(do_updates=do_updates)

    def factoryResetCb(self, do_updates=True):
        self.msg_if.pub_warn("Factory Resetting")
        self.resetControlsIFs(factory=True)
        if do_updates:
            pass
        self.initCb(do_updates=do_updates)

    # Route the app's reset paths into every ControlsIF it owns -- the per-process
    # sets and the example set alike -- as the obstacles app does.
    #
    # Both routes fall through to ControlsIF.init(), which reloads each IF's
    # controls from its own param.
    def resetControlsIFs(self, factory=False):
        controls_ifs = []
        if self.process_controls_ifs is not None:
            for process_name in self.process_controls_ifs.keys():
                controls_ifs.append(self.process_controls_ifs[process_name])
        if self.example_controls_if is not None:
            controls_ifs.append(self.example_controls_if)
        for controls_if in controls_ifs:
            if controls_if is None:
                continue
            if factory == True:
                controls_if.factory_reset()
            else:
                controls_if.reset()


    ###################
    ## Controls IF Setup
    #
    # node_if is left as None on every instance so each builds and owns its own
    # NodeClassIF -- the current device-IF convention, and what the sandbox app
    # does. Every registry and param key a ControlsIF creates is already prefixed
    # from its own controls_name (system_if.py builds node_if_prefix that way), so
    # several instances in one node cannot overwrite each other's entries or this
    # app's own -- the collision the 2026-07 DECISION LOG entry warns about.

    def setupControlsIFs(self):
        self.setupProcessControlsIFs()
        self.setupExampleControlsIF()

    # One ControlsIF per process in stereo_settings.PROCESSES_DICT, named after that
    # process. Called at startup and again on reload_processes, so it must be
    # idempotent: a process already holding an IF keeps it, because releasing one is
    # not possible (see reloadProcessesCb).
    #
    # ONE PER PROCESS, not one shared set. Each process authors its own control set
    # -- bm_1 has no convert_to_grayscale and a different block_size option list --
    # and only one process is active at a time, so the operator is shown just the
    # active one. The alternative, a single union set with the inactive half hidden,
    # cannot work: hiding a control at runtime goes through
    # nepi_controls.set_control_hidden(), which does `hidden = str(hidden)` and so
    # writes 'True'/'False' into a field nepi_interfaces/Control declares bool. That
    # string breaks the ControlsStatus publish and never satisfies
    # Nepi_IF_Controls.js's `control_msg.hidden === true` test either. A control's
    # 'hidden' works only as authored in the init dict, which is a startup value.
    # What the operator actually sees is the RUI mounting ONE Nepi_IF_Controls, on
    # the active_controls_namespace this node publishes.
    #
    # The process SELECTOR is not among them and is deliberately not a control. It
    # stays this app's own state on this app's own topics -- set_selected_process
    # (String, the process NAME) and reload_processes (Empty) -- because which
    # process runs outlives any one control set, a reload trigger is unreachable
    # through Nepi_IF_Controls (it publishes UpdateString to a topic ControlsIF
    # subscribes as UpdateTrigger), and a selector on a plain topic still works when
    # a ControlsIF does not. Turning it into a Selection control was tried in the
    # obstacles migration and dropped.
    def setupProcessControlsIFs(self):
        if self.process_controls_ifs is None:
            self.process_controls_ifs = dict()
        for process_name in self.available_processes:
            if process_name in self.process_controls_ifs.keys():
                continue
            controls_if = ControlsIF(
                        controls_name = process_name,
                        controls_display_name = process_name,
                        controls_description = 'Controls for stereo process ' + str(process_name),
                        controls_init_dict = stereo_settings.PROCESSES_DICT[process_name]['default_controls_dict'],
                        controls_updated_callback = self.makeProcessControlsUpdatedCb(process_name),
                        show_controls = True,
                        has_show_control = False,
                        log_name = process_name,
                        msg_if = self.msg_if)
            controls_if.wait_for_controls_ready()
            self.process_controls_ifs[process_name] = controls_if

    # Per-process controls callback, bound to the process it belongs to.
    #
    # ControlsIF hands its updated-callback only a control name, with no indication
    # of which instance it came from, and both process sets deliberately use the
    # same natural key names (block_size is in both). So each process IF gets its
    # own bound callback carrying the process name rather than one shared callback
    # that would have to guess which set moved.
    def makeProcessControlsUpdatedCb(self, process_name):
        def updatedCb(control_name):
            self.processControlsUpdatedCb(process_name, control_name)
        return updatedCb

    def processControlsUpdatedCb(self, process_name, control_name):
        controls_if = None
        if self.process_controls_ifs is not None:
            controls_if = self.process_controls_ifs.get(process_name, None)
        value = None
        if controls_if is not None:
            values = self.getControlsValues(controls_if)
            if values is not None:
                value = values.get(control_name, None)
        self.msg_if.pub_info("Process '" + str(process_name) + "' control '" +
                             str(control_name) + "' updated to: " + str(value))

    # Flat {control_name: current_value} snapshot of one ControlsIF, or None when
    # that IF has no controls dict to read. This is the ONLY read path the depth
    # loop uses -- see updateDepthMap().
    #
    # The None case used to be routine: ControlsIF.init() read its persisted
    # controls with get_param('controls_dict') while the param is registered under
    # '/<controls_name>_controls_dict', and ParamsIF.get_param() returns None for a
    # name it does not know -- so every config init, reset() and factory_reset()
    # replaced that IF's controls dict with None. Fixed in nepi_api/system_if.py
    # (prefixed key, plus a guard so a missing param cannot overwrite a live dict).
    # The check stays because a None dict would make
    # nepi_controls.get_control_value() raise rather than return a default, and the
    # caller, not this helper, decides what a controls-less depth pass means.
    def getControlsValues(self, controls_if):
        if controls_if is None:
            return None
        controls_dict = controls_if.get_controls_dict()
        if controls_dict is None:
            return None
        values = dict()
        for control_name in controls_dict.keys():
            values[control_name] = controls_if.get_control_value(control_name)
        return values

    # Fully-qualified namespace of one of this app's ControlsIF instances.
    #
    # Built from self.node_namespace rather than read off
    # ControlsIF.get_namespace(). That method returns
    # create_namespace(node_NAME, controls_name) -- 'app_stereo_cam/sgbm_1', with no
    # leading slash. It resolves correctly where the IF itself uses it, since rospy
    # resolves a relative name against the node's parent namespace, but this app
    # publishes the value for the RUI, which appends '/status' and hands it to
    # rosbridge -- where a name with no leading slash resolves at the global root
    # instead of under /<prefix>/<device_id>.
    def getControlsNamespace(self, controls_name):
        return nepi_sdk.create_namespace(self.node_namespace, controls_name)

    def getControlsReadyState(self):
        if self.process_controls_ifs is None:
            return False
        for process_name in self.available_processes:
            controls_if = self.process_controls_ifs.get(process_name, None)
            if controls_if is None:
                return False
            if controls_if.get_controls_ready_state() is not True:
                return False
        return True

    ###################
    ## Example Controls
    #
    # A copy of nepi_app_controls_sandbox's demonstration control set, rendered as
    # the Example Controls box at the bottom of this app's RUI column. It drives
    # nothing here: the values are logged when they change and read by nothing else,
    # so the widgets, persistence and update callbacks behave exactly as they do on
    # the sandbox page. Not a process control set, so it is kept out of
    # process_controls_ifs: it must not be reachable through
    # getControlsNamespace(selected_process) or counted in getControlsReadyState().
    # It joins the per-process IFs only in resetControlsIFs().
    #
    # Idempotent for the same reason setupProcessControlsIFs() is: setupControlsIFs()
    # runs again on reload_processes, and releasing a ControlsIF is not possible.

    def setupExampleControlsIF(self):
        if self.example_controls_if is not None:
            return
        self.example_controls_if = ControlsIF(
                    controls_name = EXAMPLE_CONTROLS_NAME,
                    controls_display_name = 'Example Controls',
                    controls_description = 'One control of every supported type',
                    controls_init_dict = self.createExampleControlsInitDict(),
                    controls_updated_callback = self.exampleControlsUpdatedCb,
                    show_controls = True,
                    has_show_control = False,
                    log_name = EXAMPLE_CONTROLS_NAME,
                    msg_if = self.msg_if)
        self.example_controls_if.wait_for_controls_ready()

    # One entry per CONTROL_TYPE, copied from controls_sandbox_app_node.py
    # createControlsInitDict() so the box renders identically to the sandbox page.
    # Insertion order sets the display order.
    #
    # demo_floats_slider is carried for fidelity with the sandbox but does not reach
    # the RUI: nepi_controls' FloatSliders branch references an undefined name and
    # silently drops the control. It is dropped on the sandbox page too.
    def createExampleControlsInitDict(self):
        controls_init_dict = {
            'demo_menu': {
                'type': 'Menu', 'default': 1, 'options': ['Off', 'Low', 'High'],
                'display_name': 'Demo Menu', 'description': 'Pick one menu option (index based).', 'hidden': False},

            'demo_selection': {
                'type': 'Selection', 'default': 'Bravo', 'options': ['Alpha', 'Bravo', 'Charlie'],
                'display_name': 'Demo Selection', 'description': 'Select a single option by name.', 'hidden': False},

            'demo_selections': {
                'type': 'Selections', 'default': ['Red', 'Blue'], 'options': ['Red', 'Green', 'Blue'],
                'display_name': 'Demo Selections', 'description': 'Select any number of options.', 'hidden': False},

            'demo_trigger': {
                'type': 'Trigger', 'default': 0,
                'display_name': 'Demo Trigger', 'description': 'Fire a one-shot trigger.', 'hidden': False},

            'demo_bool': {
                'type': 'Bool', 'default': True,
                'display_name': 'Demo Bool', 'description': 'Toggle a boolean on or off.', 'hidden': False},

            'demo_string': {
                'type': 'String', 'default': 'hello nepi',
                'display_name': 'Demo String', 'description': 'Free-form text value.', 'hidden': False},

            'demo_int': {
                'type': 'Int', 'default': 5, 'bounds': [0, 10],
                'display_name': 'Demo Int', 'description': 'Integer value within [0, 10].', 'hidden': False},

            'demo_float': {
                'type': 'Float', 'default': 2.5, 'bounds': [0.0, 10.0], 'round_value': 2,
                'display_name': 'Demo Float', 'description': 'Float value within [0.0, 10.0].', 'hidden': False},

            'demo_float_slider': {
                'type': 'FloatSlider', 'default': 50.0, 'bounds': [0.0, 100.0], 'round_value': 1,
                'display_name': 'Demo Float Slider', 'description': 'Single-value slider over [0, 100].', 'hidden': False},

            'demo_floats_slider': {
                'type': 'FloatSliders', 'default': [0.25, 0.75], 'bounds': [0.0, 1.0], 'round_value': 2,
                'display_name': 'Demo Floats Slider', 'description': 'Dual-value range slider (0.0-1.0 ratio).', 'hidden': False},
        }
        return controls_init_dict

    def exampleControlsUpdatedCb(self, control_name):
        # Called by ControlsIF after a control value/display change is applied.
        value = None
        if self.example_controls_if is not None:
            value = self.example_controls_if.get_control_value(control_name)
        self.msg_if.pub_info("Example control '" + str(control_name) + "' updated to: " + str(value))

    # Fully-qualified namespace of the example ControlsIF -- same construction, and
    # the same reason for not using ControlsIF.get_namespace(), as every other
    # controls namespace this app reports. See getControlsNamespace().
    def getExampleControlsNamespace(self):
        return self.getControlsNamespace(EXAMPLE_CONTROLS_NAME)

    def getExampleControlsReadyState(self):
        if self.example_controls_if is None:
            return False
        return self.example_controls_if.get_controls_ready_state()


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

        # Controls namespaces, all fully qualified -- see getControlsNamespace() for
        # why they are not ControlsIF.get_namespace(). The RUI mounts exactly one
        # Nepi_IF_Controls, on active_controls_namespace, which is how a node owning
        # one controls namespace per process still shows the operator only the
        # active process's controls.
        status_msg.active_controls_namespace = 'None'
        if self.selected_process in self.available_processes:
            status_msg.active_controls_namespace = self.getControlsNamespace(self.selected_process)
        controls_namespaces = []
        for process_name in self.available_processes:
            controls_namespaces.append(self.getControlsNamespace(process_name))
        status_msg.controls_namespaces = controls_namespaces
        status_msg.controls_ready = self.getControlsReadyState()

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

        # Example controls namespace, fully qualified -- see
        # getExampleControlsNamespace() for why it is not ControlsIF.get_namespace().
        # Reported before the IF exists too: initCb() publishes status while
        # NodeClassIF is still being constructed, ahead of setupExampleControlsIF().
        status_msg.example_controls_namespace = self.getExampleControlsNamespace()
        status_msg.example_controls_ready = self.getExampleControlsReadyState()

        if self.node_if is not None:
            self.node_if.publish_pub('status_pub', status_msg)


    #######################
    # Utility Functions
    #######################

    def cleanup_actions(self):
        self.msg_if.pub_info("CUSTOM_STEREO: Shutting down: Executing script cleanup actions")
        # The ControlsIF instances are deliberately NOT unregistered here.
        # ControlsIF.unregister() opens with a call to self.unsubscribe_topic(),
        # which no method in nepi_api/system_if.py defines, so it raises
        # AttributeError before it can release anything -- and it would raise inside
        # the shutdown handler, ahead of the cleanup below. Their pubs/subs go down
        # with the node.
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
