"""Custom stereo IDX node.

Two NEPI pieces, mirrored from the reference nodes:
  * Depth output  -> registered with IDXDeviceIF (like idx_zed_node.py). The
    interface owns all ROS publishing of the depth data product to the RUI;
    the node just supplies the getDepthMap callback.
  * Settings back/forth with the RUI -> the nepi_stereo "processes" module
    (patterned on nepi_stab_pt.py): create/update_processes_dict move a nested
    settings structure to and from the frontend, sanitized on the way back.
"""

import numpy as np
import cv2

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils

# NEPI IDX device interface
from nepi_api.device_if_idx import IDXDeviceIF 

from nepi_idx_custom_stereo.calibrate import Rectifier
from nepi_stereo import (
    PROCESSES_DICT,
    DEFAULT_PROCESS,
    create_processes_dict,
    update_processes_dict,
    get_blank_data_dict,
)


class CustomStereoNode(object):

    DEFAULT_NODE_NAME = "idx_custom_stereo"
    DEFAULT_MAX_FRAMERATE = 10.0

    # Data products this driver reports to the IDX interface.
    data_products = ["depth_map"]

    idx_if = None

    def __init__(self, left_source, right_source, calib_path=None):
        # ---- NEPI node bring-up ----
        # TODO: standard NEPI node init (see idx_zed_node.py / obstacles_app_node.py):
        #   nepi_sdk.init_node(name=self.DEFAULT_NODE_NAME)
        #   self.node_namespace = nepi_sdk.get_node_namespace()
        #   self.msg_if = MsgIF(...)

        # ---- Stereo settings (RUI back-and-forth via nepi_stereo) ----
        # One nested settings structure the RUI reads/writes; sanitized on return.
        self.stereo_processes_dict = create_processes_dict()
        self.selected_process = DEFAULT_PROCESS
        self.stereo_data_dict = get_blank_data_dict()

        # ---- Rectification ----
        # When calibrated, push the true geometry into EVERY process's settings.
        self.rectifier = None
        if calib_path is not None:
            self.rectifier = Rectifier(calib_path)
            for proc in self.stereo_processes_dict.values():
                proc["focal_length_px"] = self.rectifier.focal_length_px
                proc["baseline_mm"] = self.rectifier.baseline_mm

        # ---- Framerate throttle state ----
        self.max_framerate = self.DEFAULT_MAX_FRAMERATE
        self.dm_data_last_time = None

        # ---- Cameras ----
        # TODO: open both cameras from left_source / right_source.
        self.left_cam = None
        self.right_cam = None

        # ---- Launch the IDX interface (registers the depth callback) ----
        # Mirrors idx_zed_node.py: the interface publishes the data products to
        # the RUI; we hand it our callbacks.  TODO: fill device_info / cap /
        # factory settings as your tree requires.
        self.idx_if = IDXDeviceIF(
            data_products=self.data_products,
            getSettingsFunction=self.getSettings,
            settingUpdateFunction=self.settingUpdateFunction,
            setMaxFramerate=self.setMaxFramerate,
            getFramerate=self.getFramerate,
            getDepthMap=self.getDepthMap,
            stopDepthMapAcquisition=self.stopDepthMap,
            # TODO: device_info, capSettings, factorySettings, getColorImage, ...
        )

        # TODO: nepi_sdk.on_shutdown(self.cleanup_actions); nepi_sdk.spin()


    # Settings back-and-forth with the RUI (nepi_stereo processes pattern)
    def getSettings(self):
        """RUI read path: hand out the current nested processes settings."""
        return self.stereo_processes_dict

    def settingUpdateFunction(self, incoming_processes_dict):
        """RUI write path: sanitize incoming settings against known defaults."""
        # update_processes_dict drops unknown keys and preserves structure, so a
        # malformed RUI payload can't inject bad settings.
        self.stereo_processes_dict = update_processes_dict(incoming_processes_dict)
        return True, "stereo settings updated"

    def setSelectedProcess(self, process_name):
        if process_name in PROCESSES_DICT:
            self.selected_process = process_name
            return True
        return False

    def setMaxFramerate(self, max_framerate):
        self.max_framerate = max_framerate

    def getFramerate(self):
        return self.max_framerate


    # Frame acquisition
    def _grab_pair(self):
        """Return (ok, left_bgr, right_bgr). Frames must be time-synced."""
        # TODO: read both cameras. If not hardware-synced, depth on moving
        #       scenes will be wrong.
        #   okL, left  = self.left_cam.read()
        #   okR, right = self.right_cam.read()
        #   return (okL and okR), left, right
        raise NotImplementedError



    # Depth map (IDX contract -- see idx_zed_node.getDepthMap)
    def getDepthMap(self):
        status = False
        msg = ""
        np_depth_map = None
        timestamp = None
        encoding = '32FC1'          # matches the float32 mm depth output

        # framerate throttle
        last_time = self.dm_data_last_time
        current_time = nepi_utils.get_time()

        need_data = False
        if last_time is not None and self.idx_if is not None:
            fr_delay = float(1) / self.max_framerate
            if (current_time - last_time) > fr_delay:
                need_data = True
        else:
            need_data = True

        if need_data == False:
            return False, "Waiting for Timer", None, None, None

        # Grab a synchronized L/R pair
        ok, left, right = self._grab_pair()
        if not ok:
            return False, "Frame grab failed", None, None, None

        # Rectify (compute_depth_map assumes rectified input)
        if self.rectifier is not None:
            left, right = self.rectifier.rectify(left, right)

        # Run the selected stereo process (fills self.stereo_data_dict)
        process_fn = PROCESSES_DICT[self.selected_process]['process_function']
        settings = self.stereo_processes_dict[self.selected_process]
        self.stereo_data_dict, _ = process_fn(left, right, self.stereo_data_dict, settings)

        np_depth_map = self.stereo_data_dict['depth_map']   # (H,W) float32 mm
        status = True

        # Match the ZED stereo convention: invalid pixels -> nan instead of 0.0.
        np_depth_map[np_depth_map <= 0.0] = np.nan

        timestamp = nepi_utils.get_time()
        self.dm_data_last_time = nepi_utils.get_time()

        return status, msg, np_depth_map, timestamp, encoding

    def stopDepthMap(self):
        # TODO: any teardown when the RUI stops the depth subscription.
        pass



    # Cleanup
    def cleanup_actions(self):
        # TODO: release camera handles
        pass
