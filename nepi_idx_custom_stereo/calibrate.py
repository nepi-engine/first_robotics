#!/usr/bin/env python3
"""
CLAUDE GENERATED. NEEDS WORK BEFORE FUNCTIONAL.
Stereo calibration + rectification for the custom stereo driver.

Three jobs (all offline setup, done BEFORE running the driver):
  1. calibrate() -- once. Chessboard pairs -> saved .npz with the
     rectification maps + true focal_length_px / baseline_mm.  (skeleton)
  2. Rectifier   -- per frame. Loads the .npz and warps raw L/R
     frames so features line up on the same row for block matching.  (done)
  3. run_tuner() -- interactive sliders to find good block-match params. (skeleton)

Import and call these directly; there is no CLI.
"""

from __future__ import annotations

import numpy as np
import cv2

from stereo_library import DEFAULT_SETTINGS, compute_depth_map, colorize_depth


# Stage 2: per-frame rectification
class Rectifier:
    """Applies saved rectification maps to raw L/R frames."""

    def __init__(self, calib_path):
        data = np.load(calib_path)
        self.map1x, self.map1y = data["map1x"], data["map1y"]
        self.map2x, self.map2y = data["map2x"], data["map2y"]
        self.Q = data["Q"]
        # These two feed stereo_library settings so depth uses true geometry.
        self.focal_length_px = float(data["focal_length_px"])
        self.baseline_mm = float(data["baseline_mm"])

    def rectify(self, left_image, right_image):
        """Return (rect_left, rect_right), ready for compute_depth_map()."""
        left = cv2.remap(left_image, self.map1x, self.map1y, cv2.INTER_LINEAR)
        right = cv2.remap(right_image, self.map2x, self.map2y, cv2.INTER_LINEAR)
        return left, right


# Stage 1: calibration (skeleton -- fill in the TODOs)
def calibrate(left_glob, right_glob, cols, rows, square_mm, out_path):
    """Calibrate a stereo pair from chessboard image pairs; save the maps.

    Produces the .npz that Rectifier() loads: map1x/1y, map2x/2y, Q, and the
    scalars focal_length_px + baseline_mm.
    """
    # TODO: gather + sort the left/right image paths (glob), verify equal counts.

    # TODO: build the board's object points once:
    #   objp = np.zeros((rows*cols, 3), np.float32)
    #   objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    #   objp *= square_mm            # -> T/baseline come out in mm

    # TODO: per image pair, find + refine corners; keep pairs where BOTH found:
    #   cv2.findChessboardCorners(gray, (cols, rows))
    #   cv2.cornerSubPix(...)
    #   -> accumulate objpoints, imgpoints_l, imgpoints_r; record image_size (w,h)

    # TODO: per-camera intrinsics:
    #   cv2.calibrateCamera(objpoints, imgpoints_l, image_size, None, None) -> KL, DL
    #   cv2.calibrateCamera(objpoints, imgpoints_r, image_size, None, None) -> KR, DR

    # TODO: stereo pose (hold intrinsics fixed), print the RMS error (<1 is good):
    #   cv2.stereoCalibrate(..., flags=cv2.CALIB_FIX_INTRINSIC) -> R, T

    # TODO: rectification transforms:
    #   cv2.stereoRectify(KL, DL, KR, DR, image_size, R, T,
    #                     flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
    #     -> R1, R2, P1, P2, Q, roi1, roi2

    # TODO: precompute the remap lookup tables (what makes per-frame rectify cheap):
    #   cv2.initUndistortRectifyMap(KL, DL, R1, P1, image_size, cv2.CV_32FC1) -> map1x, map1y
    #   cv2.initUndistortRectifyMap(KR, DR, R2, P2, image_size, cv2.CV_32FC1) -> map2x, map2y

    # TODO: extract geometry (these go into DEFAULT_SETTINGS):
    #   focal_length_px = P1[0, 0]
    #   baseline_mm     = -P2[0, 3] / P2[0, 0]     # P2[0,3] = -fx * baseline

    # TODO: np.savez(out_path, map1x=..., map1y=..., map2x=..., map2y=...,
    #                Q=..., focal_length_px=..., baseline_mm=...)
    raise NotImplementedError


# Stage 3: interactive block-match tuning (skeleton -- fill in the TODOs)
WINDOW = "tuner"

# Each entry: (slider_name, initial_value, max_value)
# TODO: pick sensible max values for each.
TRACKBARS = [
    ("num_disparities",     128, 256),
    ("block_size",            5,  21),
    ("uniqueness_ratio",     10,  50),
    ("speckle_window_size", 100, 200),
    ("speckle_range",         2, 100),
    ("disp12_max_diff",       1,  25),   # slider 0 will mean -1 (see sanitize)
    ("pre_filter_cap",       63,  63),
]


def build_settings_from_sliders():
    """Read every trackbar, sanitize, and return a settings dict."""
    # TODO: read each slider value with cv2.getTrackbarPos(name, WINDOW)

    # TODO sanitize (trackbars are int and >= 0):
    #   num_disparities  -> max(16, (v // 16) * 16)   # positive multiple of 16
    #   block_size       -> v | 1, then max(3, ...)   # force odd, >= 3
    #   disp12_max_diff  -> v - 1                      # lets you reach -1

    # TODO: return {**DEFAULT_SETTINGS, <overrides from sliders>}
    raise NotImplementedError


def run_tuner(left_path, right_path):
    # TODO: load the L/R pair once (cv2.imread). Use RECTIFIED images.

    # TODO: cv2.namedWindow(WINDOW); create every trackbar in TRACKBARS
    #       (callback can be `lambda x: None`; values are polled in the loop).

    last = None  # cache last settings so we only recompute when something changed
    while True:
        # TODO: settings = build_settings_from_sliders()
        # TODO: if settings != last: recompute depth via compute_depth_map(...)
        #       and cv2.imshow(WINDOW, colorize_depth(depth, settings)); last = settings

        key = cv2.waitKey(50) & 0xFF
        if key == ord("q"):
            break

    # TODO: print the final `settings` dict so it can be pasted into DEFAULT_SETTINGS
    cv2.destroyAllWindows()
