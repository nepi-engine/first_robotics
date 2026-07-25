#!/usr/bin/env python3
"""
Interactive tuner for the stereo block-matching parameters.

Dev tool to find good values. Not part of the driver itself.

TODO:
    - Load a rectified stereo pair.
    - Create trackbars for each parameter in TRACKBARS.
    - Poll trackbar values, sanitize, and build settings dict.
    - Recompute depth map when settings change.
    - Display colorized depth map.
    - Print final settings dict on exit.
"""

import sys

import numpy as np
import cv2

from stereo_library import DEFAULT_SETTINGS, compute_depth_map, colorize_depth


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


def main(left_path, right_path):
    # TODO: load the L/R pair once (cv2.imread). Ideally rectified images.

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


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit("usage: python3 stereo_tuner.py left.png right.png")
    main(sys.argv[1], sys.argv[2])
