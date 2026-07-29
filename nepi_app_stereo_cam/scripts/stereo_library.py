#!/usr/bin/env python3
"""Custom stereo depth library for NEPI.

Produces a metric depth map (distance in millimeters) from a rectified
left/right stereo image pair using OpenCV block matching algorithm.

Assumptions
-----------
* The two input images are the same aspect ratio / pixel resolution.
* The images are already RECTIFIED (epipolar lines are horizontal image
  rows).  Block matching relies on this.  If your raw cameras are not
  rectified, calibrate them and pass rectified frames in.
* ``focal_length_px`` and ``baseline_mm`` come from stereo calibration and
  are what turn a raw disparity into a real-world distance.
"""

from __future__ import annotations

import numpy as np
import cv2


DEFAULT_SETTINGS = {
    ## - Camera Geometry
    # Baseline: distance between the two camera optical centers, in mm.
    "baseline_mm": 60.0,
    # Focal length in px (fx from the camera matrix).
    "focal_length_px": 700.0,
    # sgbm (semi-global, higher quality) or bm (faster, noisier, grayscale only).
    "matcher": "sgbm",
    # Convert color inputs to grayscale before matching (required for BM).
    "convert_to_grayscale": True,

    # - Block matching parameters
    
    #disparity = the difference in pixel locations of said object between 2 cameras
    #minimum disparity allowed, only change if objects are significantly far away from camera, where near-zero disparities never occur.
    "min_disparity": 0,
    # Range of disparities to search.  MUST be a multiple of 16. Larger = can measure closer objects (larger disparities)
    "num_disparities": 128,
    # side length of the matching window. Odd, typically 3-11 for SGBM, 5-21 for BM.
    "block_size": 5,
    # a match must beat the second-best candidate by at least this percent margin, or it is rejected. 0 disables.
    "uniqueness_ratio": 10,
    # max size (in pixels) of a connected disparity blob that gets treated as noise and cleared. 0 disables speckle filtering.
    "speckle_window_size": 100,
    # how much disparity variation is allowed within a blob for it to count as one connected region during speckle filtering.
    "speckle_range": 2,
    # Occlusion filter: left-right consistency check (if no match between pixels,); -1 disables.
    "disp12_max_diff": 1,
    # reduces noise to prevent large disparity jumps due to image noise; higher values = retained sharper texture details; 1-63 is typical
    "pre_filter_cap": 63,

    # - Depth post-processing
    # Distances outside this range are treated as invalid (set to 0).
    "min_depth_mm": 50,
    "max_depth_mm": 20000,
    # k*k median filter applied to the depth map to remove isolated depth pixels without blurring actual edges; must be odd, 0 disables.
    "median_filter_size": 5,   
}        


# Internal helpers
def _as_grayscale(image: np.ndarray) -> np.ndarray:
    #Return a single-channel uint8 image
    if image.ndim == 3:
        return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return image

def _build_matcher(settings: dict):
    #Construct an OpenCV stereo matcher from settings
    num_disparities = int(settings["num_disparities"])
    if num_disparities % 16 != 0 or num_disparities <= 0:
        raise ValueError(
            f"num_disparities must be a positive multiple of 16, got {num_disparities}"
        )
    block_size = int(settings["block_size"])
    if block_size % 2 == 0:
        raise ValueError(f"block_size must be odd, current block_size: {block_size}")

    matcher_type = settings["matcher"].lower()
    if matcher_type == "bm":
        matcher = cv2.StereoBM_create(
            numDisparities=num_disparities,
            blockSize=block_size,
        )
        matcher.setMinDisparity(int(settings["min_disparity"]))
        matcher.setUniquenessRatio(int(settings["uniqueness_ratio"]))
        matcher.setSpeckleWindowSize(int(settings["speckle_window_size"]))
        matcher.setSpeckleRange(int(settings["speckle_range"]))
        matcher.setDisp12MaxDiff(int(settings["disp12_max_diff"]))
        matcher.setPreFilterCap(int(settings["pre_filter_cap"]))
        return matcher

    if matcher_type == "sgbm":
        channels = 1 if settings["convert_to_grayscale"] else 3
        return cv2.StereoSGBM_create(
            minDisparity=int(settings["min_disparity"]),
            numDisparities=num_disparities,
            blockSize=block_size,
            # P1/P2 control smoothness, standard heuristic from OpenCV docs. SGBM strictly requires p2 > p1, so big change in depth is penalized over small changes
            P1=8 * channels * block_size ** 2, # P1: penalty on disparity change by +/- 1 between neighboring pixels
            P2=32 * channels * block_size ** 2, # P2: penalty on disparity change by more than 1 between neighboring pixels
            disp12MaxDiff=int(settings["disp12_max_diff"]),
            uniquenessRatio=int(settings["uniqueness_ratio"]),
            speckleWindowSize=int(settings["speckle_window_size"]),
            speckleRange=int(settings["speckle_range"]),
            preFilterCap=int(settings["pre_filter_cap"]),
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )

    raise ValueError(f"Unknown matcher '{settings['matcher']}' (use 'sgbm' or 'bm')")


def _disparity_to_depth_mm(disparity: np.ndarray, settings: dict) -> np.ndarray:
    # Z_mm = focal_length_px * baseline_mm / disparity_px
    
    focal_px = np.float32(settings["focal_length_px"])
    baseline_mm = np.float32(settings["baseline_mm"])

    depth = np.zeros_like(disparity, dtype=np.float32)
    valid = disparity > 0  # non-positive disparity => no/invalid match
    depth[valid] = (focal_px * baseline_mm) / disparity[valid]

    # Clamp to the configured usable range; out-of-range = invalid (0).
    min_mm = float(settings["min_depth_mm"])
    max_mm = float(settings["max_depth_mm"])
    depth[(depth < min_mm) | (depth > max_mm)] = 0.0
    return depth



# Main entry point
def compute_depth_map(left_image: np.ndarray, right_image: np.ndarray, settings: dict = None,) -> np.ndarray:
    """Compute a metric depth map from a rectified stereo pair.
    Parameters:

    left_image, right_image : np.ndarray
        cv2 images (BGR or grayscale) from the LEFT and RIGHT cameras.
        Must have identical height / width (resolution).
    settings : dict, optional
        User settings; missing keys are filled from DEFAULT_SETTINGS dict.

    Returns:

    np.ndarray
        (H, W) float32 matrix.  Each entry is the distance to that
        pixel in mm; 0.0 marks pixels with no valid depth.
    """
    if settings is None:
        settings = {}
    # Merge user overrides onto defaults so a partial dict is fine.
    cfg = {**DEFAULT_SETTINGS, **settings}

    if left_image is None or right_image is None:
        raise ValueError("left_image and right_image must both be provided")
    if left_image.shape[:2] != right_image.shape[:2]:
        raise ValueError(
            f"Left/right resolution mismatch: "
            f"{left_image.shape[:2]} vs {right_image.shape[:2]}"
        )

    left = right = None
    if cfg["convert_to_grayscale"] or cfg["matcher"].lower() == "bm":
        left = _as_grayscale(left_image)
        right = _as_grayscale(right_image)
    else:
        left, right = left_image, right_image

    matcher = _build_matcher(cfg)

    
    # BM/SGBM return disparity as int16 fixed-point scaled by 16.
    raw_disparity = matcher.compute(left, right)
    disparity = raw_disparity.astype(np.float32) / 16.0

    depth_mm = _disparity_to_depth_mm(disparity, cfg)

    ksize = int(cfg["median_filter_size"])
    if ksize >= 3 and ksize % 2 == 1:
        depth_mm = cv2.medianBlur(depth_mm, ksize)

    return depth_mm



#colorized preview for humans (not part of the depth output)
def colorize_depth(depth_mm: np.ndarray, settings: dict = None) -> np.ndarray:
    """Return a BGR uint8 image visualizing the depth map (for debugging)."""
    cfg = {**DEFAULT_SETTINGS, **(settings or {})}
    min_mm, max_mm = float(cfg["min_depth_mm"]), float(cfg["max_depth_mm"])
    norm = np.clip((depth_mm - min_mm) / max(max_mm - min_mm, 1e-6), 0.0, 1.0)
    norm_u8 = (norm * 255).astype(np.uint8)
    colored = cv2.applyColorMap(norm_u8, cv2.COLORMAP_JET)
    colored[depth_mm <= 0.0] = 0  # black = invalid
    return colored
