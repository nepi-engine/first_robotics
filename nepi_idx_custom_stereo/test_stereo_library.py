#!/usr/bin/env python3
"""
CLAUDE GENERATED. NOT READY FOR USE.

Test harness for stereo_library.compute_depth_map.

Usage
-----
    # Test with two real rectified images:
    python3 test_stereo_library.py left.png right.png

    # No args -> generate a synthetic pair with a KNOWN disparity and
    # verify the recovered depth matches the expected distance.
"""

import sys

import numpy as np
import cv2

from stereo_library import DEFAULT_SETTINGS, compute_depth_map, colorize_depth


def make_synthetic_pair(width=640, height=480, shift_px=32):
    """Create a fake rectified stereo pair.

    A textured square is drawn on both images but shifted horizontally by
    ``shift_px`` in the RIGHT image.  That horizontal shift IS the
    disparity, so we can predict the depth the library should recover.
    """
    rng = np.random.default_rng(0)
    background = rng.integers(0, 60, size=(height, width, 3), dtype=np.uint8)
    left = background.copy()
    right = background.copy()

    # A textured foreground patch (so block matching has features to lock).
    patch = rng.integers(150, 255, size=(160, 160, 3), dtype=np.uint8)
    y0, x0 = 160, 240
    left[y0:y0 + 160, x0:x0 + 160] = patch
    # Object nearer the camera -> appears further LEFT in the right image.
    right[y0:y0 + 160, x0 - shift_px:x0 - shift_px + 160] = patch
    return left, right


def run_synthetic():
    shift_px = 32
    left, right = make_synthetic_pair(shift_px=shift_px)

    settings = dict(DEFAULT_SETTINGS)
    settings["num_disparities"] = 64
    settings["block_size"] = 7

    depth = compute_depth_map(left, right, settings)

    # Predicted depth for the known disparity:
    expected_mm = (settings["focal_length_px"] * settings["baseline_mm"]) / shift_px

    # Sample the center of the foreground patch.
    patch_depths = depth[200:280, 280:360]
    valid = patch_depths[patch_depths > 0]

    print("=== Synthetic stereo test ===")
    print(f"image size            : {left.shape[1]}x{left.shape[0]}")
    print(f"depth matrix shape    : {depth.shape} (dtype {depth.dtype})")
    print(f"known disparity        : {shift_px} px")
    print(f"expected patch depth   : {expected_mm:.1f} mm")
    if valid.size:
        measured = float(np.median(valid))
        err = abs(measured - expected_mm) / expected_mm * 100.0
        print(f"measured patch depth   : {measured:.1f} mm  ({err:.1f}% error)")
        assert err < 15.0, "recovered depth is off by more than 15%"
        print("PASS: recovered depth matches expected within tolerance.")
    else:
        raise AssertionError("no valid depth recovered in patch region")

    cv2.imwrite("depth_preview.png", colorize_depth(depth, settings))
    print("wrote depth_preview.png")


def run_real(left_path, right_path):
    left = cv2.imread(left_path, cv2.IMREAD_COLOR)
    right = cv2.imread(right_path, cv2.IMREAD_COLOR)
    if left is None or right is None:
        sys.exit(f"Could not read images: {left_path!r}, {right_path!r}")

    depth = compute_depth_map(left, right, DEFAULT_SETTINGS)
    valid = depth[depth > 0]

    print("=== Real stereo pair ===")
    print(f"depth matrix shape    : {depth.shape} (dtype {depth.dtype})")
    print(f"valid pixels           : {valid.size} / {depth.size} "
          f"({100.0 * valid.size / depth.size:.1f}%)")
    if valid.size:
        print(f"depth range            : {valid.min():.1f} .. {valid.max():.1f} mm")
        print(f"median depth           : {np.median(valid):.1f} mm")

    cv2.imwrite("depth_preview.png", colorize_depth(depth, DEFAULT_SETTINGS))
    print("wrote depth_preview.png")


if __name__ == "__main__":
    if len(sys.argv) == 3:
        run_real(sys.argv[1], sys.argv[2])
    else:
        run_synthetic()
