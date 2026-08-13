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

import copy
import math

import numpy as np
import cv2

from nepi_sdk import nepi_utils
from nepi_sdk import nepi_controls

from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_obstacles"
logger = Logger(log_name = log_name)


########################
## Library Data

# Control definitions in the nepi_controls init-dict form. A node deep-copies
# this and hands it to a ControlsIF, which turns it into a live controls dict
# and passes that dict back on every process cycle.
#
# Every default must sit inside its own bounds. nepi_controls.create_controls_dict
# raises NameError on an out-of-bounds Float default and the bare except around
# the per-control body silently DROPS the control, so an out-of-bounds default
# does not clamp -- it makes the control disappear.
PROCESS_CONTROLS_DICT = {

    'min_range_m': {
        'type': 'Float', 'default': 0.5, 'bounds': [0.1, 100.0], 'round_value': 2,
        'display_name': 'Min Range (m)',
        'description': 'Ignore depth map returns closer than this range.',
        'hidden': False},

    'max_range_m': {
        'type': 'Float', 'default': 10.0, 'bounds': [0.2, 100.0], 'round_value': 2,
        'display_name': 'Max Range (m)',
        'description': 'Ignore depth map returns farther than this range.',
        'hidden': False},

    'no_return_range_m': {
        'type': 'Float', 'default': 0.0, 'bounds': [0.0, 200.0], 'round_value': 2,
        'display_name': 'No Return Range (m)',
        'description': 'Range a source reports for a pixel that got no return. '
                       'Zero detects it from the depth map; set above the '
                       'sensor range to keep every return.',
        'hidden': False},

    'ground_max_height_m': {
        'type': 'Float', 'default': -0.3, 'bounds': [-10.0, 10.0], 'round_value': 2,
        'display_name': 'Ground Height (m)',
        'description': 'Returns at or below this height relative to the sensor are ground, not obstacles.',
        'hidden': False},

    'obstacle_max_height_m': {
        'type': 'Float', 'default': 100.0, 'bounds': [-10.0, 100.0], 'round_value': 2,
        'display_name': 'Obstacle Max Height (m)',
        'description': 'Returns above this height relative to the sensor are overhead, '
                       'not obstacles. Leave at 100 for no limit.',
        'hidden': False},

    'range_step_m': {
        'type': 'Float', 'default': 0.5, 'bounds': [0.05, 20.0], 'round_value': 2,
        'display_name': 'Range Step (m)',
        'description': 'Neighbouring returns further apart than this in range belong to separate obstacles.',
        'hidden': False},

    'min_obstacle_size_ratio': {
        'type': 'FloatSlider', 'default': 0.02, 'bounds': [0.0, 1.0], 'round_value': 3,
        'display_name': 'Min Obstacle Size',
        'description': 'Smallest obstacle to report, as a fraction of the depth map area.',
        'hidden': False},

    'max_obstacles': {
        'type': 'Int', 'default': 10, 'bounds': [1, 50],
        'display_name': 'Max Obstacles',
        'description': 'Report at most this many obstacles per cycle, largest first.',
        'hidden': False},

    'use_navpose': {
        'type': 'Bool', 'default': True,
        'display_name': 'Use NavPose',
        'description': 'Level the height calculation using the connected NavPose roll and pitch.',
        'hidden': False},

    'mount_pitch_deg': {
        'type': 'Float', 'default': 0.0, 'bounds': [-90.0, 90.0], 'round_value': 2,
        'display_name': 'Mount Pitch (deg)',
        'description': 'Sensor pitch on its mount, positive nose up. Added to the NavPose pitch.',
        'hidden': False},

    'mount_roll_deg': {
        'type': 'Float', 'default': 0.0, 'bounds': [-90.0, 90.0], 'round_value': 2,
        'display_name': 'Mount Roll (deg)',
        'description': 'Sensor roll on its mount, positive right side down. Added to the NavPose roll.',
        'hidden': False},

    }


# Per-cycle process bookkeeping, keyed by source depth map topic. Tracking pixel
# velocity needs the previous cycle's obstacle centroids, and a module-level
# store is how the process functions in this SDK keep that between calls. One
# node process owns one obstacles process, so there is no cross-process sharing.
PROCESS_DATA_DICT = {
    # Required Fields
    'data_time': 0.0,
    'process_time': 0.0,
    # source topic -> {'time': float, 'obstacles': [ {'uid','cx','cy'} ]}
    'last_cycle': {},
    'next_id': 0,
    # Add Fields
}

# An obstacle centroid within this fraction of the image diagonal of a previous
# centroid is treated as the same obstacle for uid and velocity continuity.
TRACK_MATCH_RATIO = 0.10

# Fallback field of view when the source status message reports none.
DEFAULT_WIDTH_DEG = 110.0
DEFAULT_HEIGHT_DEG = 70.0

FLOAT_FIELD_UNSET = -999.0
INT_FIELD_UNSET = -999

# NEPI depth map sources do NOT mark a no-return pixel with NaN, inf or zero.
# They CLAMP it to the sensor's max range, so a pixel that saw nothing arrives
# as a perfectly ordinary far measurement and there is no value in the array
# that distinguishes the two. Measured: every frame of the TUM pioneer_360 set
# carries 14.7-68.1% of its pixels at exactly 10000.0 mm with zero NaN, zero
# inf and zero zeros, and a zed2 capture carries ~13% at exactly 20000.0 mm.
# nepi_img.npDepthMap_to_cv2ColorImg agrees from the other side: it folds NaN,
# inf and below-min all onto the max-range colour.
#
# Left in, that plateau is a large surface at the far range gate, and the
# height threshold cuts it along the fixed row where range*sin(el) crosses
# ground_max_height_m -- one phantom obstacle above that row, phantom ground
# below it, in the same place on every frame regardless of scene.
#
# The sentinel value is a per-sensor constant this module cannot know, so it is
# detected per frame: a clamp is the frame's maximum AND holds an implausible
# share of the frame at one exact float. Measured over 149 consecutive frames,
# the maximum holds 18.2-68.1% of the frame while the clamp is present and
# 0.0003-0.40% once it is removed -- a 45x gap at worst, so the threshold below
# sits an order of magnitude clear on both sides.
#
# A source whose no-returns are too sparse to form a plateau falls under this
# threshold and stays in. Those pixels are by definition too few to reach
# min_obstacle_size_ratio, and the no_return_range_m control names the value
# outright for any source where the detection is not wanted.
NO_RETURN_MIN_RATIO = 0.02

# A declared or detected sentinel is compared with a hair of margin rather than
# for equality, so a source that quantizes or round-trips its clamp through a
# different dtype still lands on it. One part in ten thousand, which gives up
# the last millimetre of a 10 m sensor.
NO_RETURN_TOLERANCE_RATIO = 1.0e-4

# NEPI depth map data products carry MILLIMETRE range values while their status
# bounds and every control in this module are in METRES. nepi_img is the
# authority on this split: get_range_from_npDepthMap divides the array by 1000
# to report metres, and npDepthMap_to_cv2ColorImg scales its metre bounds up by
# 1e3 to compare against the array. Producers convert up to millimetres for
# exactly this reason (see idx_zed_node and stereo_cam_app_node). This module
# does its geometry in metres and hands the segmentation maps back in the
# source's millimetres, so the same colorizer renders them.
MM_PER_M = 1000.0

# Per-pixel bearing and sensor-frame unit vectors depend only on the depth map
# dimensions and the source field of view, never on the frame data, so they are
# built once per geometry and reused. Rebuilding them cost two np.tile
# allocations and four full-frame trig passes on every cycle. A source that
# never changes resolution or field of view builds this once for its lifetime.
GEOMETRY_CACHE = {}
MAX_GEOMETRY_CACHE_LEN = 4


#########################
# Process Functions
#########################





def process_results(np_depth_map, status_dict, navpose_dict, data_dict, controls_dict):
    """Segment a depth map into ground and obstacles and locate each obstacle.

    Range-gates the depth map, converts every remaining return to a height in a
    gravity-levelled sensor frame, splits the returns into ground and obstacle
    sets on a height threshold, groups the obstacle returns into connected
    components, and reports one entry per component with its pixel box, range,
    azimuth, elevation, and pixel velocity.

    Args:
        np_depth_map (numpy.ndarray): Range image in MILLIMETRES, shape (H, W).
            Non-returns may be NaN, inf or zero, or -- as every NEPI depth map
            source actually encodes them -- clamped to the sensor's max range.
            A clamped plateau is detected and rejected per frame; the
            no_return_range_m control names the value outright when it is
            known. See MM_PER_M and NO_RETURN_MIN_RATIO.
        status_dict (dict): The source's DepthMapStatus as a dictionary. Read
            for 'width_deg', 'height_deg' and 'depth_map_topic'.
        navpose_dict (dict): The source's NavPose as a dictionary. Read for
            'has_orientation', 'roll_deg' and 'pitch_deg'.
        data_dict (dict): Per-cycle process bookkeeping in the
            PROCESS_DATA_DICT form. Returned with its timing fields updated.
        controls_dict (dict): Live nepi_controls controls dictionary built from
            PROCESS_CONTROLS_DICT.

    Returns:
        list: obstacles_dict_list -- one dict per obstacle, ordered largest
            first, carrying every field of nepi_app_obstacles/Obstacle.
        numpy.ndarray: depth_map_ground -- the range-gated depth map in the
            source's millimetres with every non-ground pixel set to NaN, or
            None if no ground was found.
        numpy.ndarray: depth_map_obstacles -- the range-gated depth map in the
            source's millimetres with every non-obstacle pixel set to NaN, or
            None if no obstacle returns were found.
        dict: data_dict -- the caller's bookkeeping dict.

    Every return path hands back all four values. The caller unpacks four, and
    a short tuple raises a ValueError that its try/except swallows, which loses
    the whole cycle including any segmentation map already built.
    """
    obstacles_dict_list = []
    depth_map_ground = None
    depth_map_obstacles = None

    start_time = nepi_utils.get_time()

    if np_depth_map is None:
        return obstacles_dict_list, depth_map_ground, depth_map_obstacles, data_dict

    try:
        np_depth_map = np.asarray(np_depth_map, dtype = np.float32)
        if np_depth_map.ndim != 2:
            logger.log_warn("Depth map is not a single channel range image: " + str(np_depth_map.shape), throttle_s = 5.0)
            return obstacles_dict_list, depth_map_ground, depth_map_obstacles, data_dict

        min_range_m = getControlValue(controls_dict, 'min_range_m', 0.5)
        max_range_m = getControlValue(controls_dict, 'max_range_m', 10.0)
        set_no_return_range_m = getControlValue(controls_dict, 'no_return_range_m', 0.0)
        ground_max_height_m = getControlValue(controls_dict, 'ground_max_height_m', -0.3)
        obstacle_max_height_m = getControlValue(controls_dict, 'obstacle_max_height_m', 100.0)
        range_step_m = getControlValue(controls_dict, 'range_step_m', 0.5)
        min_obstacle_size_ratio = getControlValue(controls_dict, 'min_obstacle_size_ratio', 0.02)
        max_obstacles = int(getControlValue(controls_dict, 'max_obstacles', 10))
        use_navpose = getControlValue(controls_dict, 'use_navpose', True)
        mount_pitch_deg = getControlValue(controls_dict, 'mount_pitch_deg', 0.0)
        mount_roll_deg = getControlValue(controls_dict, 'mount_roll_deg', 0.0)

        if max_range_m <= min_range_m:
            max_range_m = min_range_m + 0.1
        # Same shape as the range clamp above: an upper bound at or under the
        # lower one would leave nothing an obstacle at all, which reads in the
        # RUI as the process having died rather than as a control being wrong.
        if obstacle_max_height_m <= ground_max_height_m:
            obstacle_max_height_m = ground_max_height_m + 0.1

        height_px, width_px = np_depth_map.shape
        width_deg = float(status_dict.get('width_deg', DEFAULT_WIDTH_DEG) or DEFAULT_WIDTH_DEG)
        height_deg = float(status_dict.get('height_deg', DEFAULT_HEIGHT_DEG) or DEFAULT_HEIGHT_DEG)
        source_topic = str(status_dict.get('depth_map_topic', '') or '')

        ##############################
        # Range gate.
        #
        # The source array is millimetres; every control and every geometry
        # term below is metres, so convert once here and keep np_depth_map as
        # the untouched source-unit copy the segmentation maps are cut from.
        np_ranged = np_depth_map / MM_PER_M

        # No-return pixels first, because they are indistinguishable from real
        # far measurements and the range gate below would otherwise admit the
        # whole plateau as a surface at max_range_m. Rejected here rather than
        # by pulling max_range_m down under the sentinel, which is what an
        # operator has to do today: that is a workaround, it silently costs the
        # top of the real range, and it does nothing on a source whose sentinel
        # sits above the range the operator wanted anyway.
        no_return_range_m = getNoReturnRange(np_ranged, set_no_return_range_m)

        # One comparison pass rather than five. min_range_m is bounded at 0.1,
        # so the gate subsumes the old zero and infinity tests: NaN and -inf
        # both fail the lower bound, +inf fails the upper, and every comparison
        # against NaN is False. The kept interval is identical to the one the
        # four separate assignments left behind.
        with np.errstate(invalid = 'ignore'):
            valid_mask = (np_ranged >= min_range_m) & (np_ranged <= max_range_m)
            if no_return_range_m is not None:
                valid_mask &= (np_ranged < no_return_range_m)
        if valid_mask.any() == False:
            return obstacles_dict_list, depth_map_ground, depth_map_obstacles, data_dict
        np.copyto(np_ranged, np.float32(np.nan), where = np.logical_not(valid_mask))

        ##############################
        # Per-pixel bearing and height
        geometry_dict = getGeometry(width_px, height_px, width_deg, height_deg)
        az_deg_map = geometry_dict['az_deg_map']
        el_deg_map = geometry_dict['el_deg_map']
        [roll_deg, pitch_deg] = getLevelAngles(navpose_dict, use_navpose, mount_roll_deg, mount_pitch_deg)
        np_height = getHeightMap(np_ranged, geometry_dict, roll_deg, pitch_deg)

        ##############################
        # Ground / obstacle split
        # Three outcomes, not two. A return above obstacle_max_height_m is
        # OVERHEAD -- a ceiling, a rafter, the upper courses of a wall, a
        # gantry the vehicle drives under -- and is neither ground nor an
        # obstacle, so it appears in neither segmentation map and cannot join
        # a component. Until now there was no upper bound anywhere in the
        # package and every one of those was reported as an obstacle.
        ground_mask = valid_mask & (np_height <= ground_max_height_m)
        obstacle_mask = valid_mask & (np_height > ground_max_height_m) & (np_height <= obstacle_max_height_m)

        # Cut both maps from the SOURCE array, not from np_ranged, so they come
        # back in the millimetres every downstream consumer of a NEPI depth map
        # expects -- the colorizer included. Non-member pixels stay NaN, which
        # is the marker the overlay and the segmentation viewers mask on.
        if ground_mask.any():
            depth_map_ground = np.full(np_depth_map.shape, np.nan, dtype = np.float32)
            depth_map_ground[ground_mask] = np_depth_map[ground_mask]
        if obstacle_mask.any():
            depth_map_obstacles = np.full(np_depth_map.shape, np.nan, dtype = np.float32)
            depth_map_obstacles[obstacle_mask] = np_depth_map[obstacle_mask]
        else:
            # No obstacle returns is a normal outcome, and the ground map built
            # just above is still valid -- hand it back rather than dropping it.
            recordCycle(source_topic, [], start_time)
            data_dict['data_time'] = start_time
            data_dict['process_time'] = nepi_utils.get_time() - start_time
            return obstacles_dict_list, depth_map_ground, depth_map_obstacles, data_dict

        ##############################
        # Group obstacle returns.
        #
        # Height thresholding alone puts everything standing above the ground
        # into one connected region, so a near object and the wall behind it
        # merge into a single blob. Breaking connectivity across range
        # discontinuities is what separates them, and it keeps a continuous
        # surface whole -- unlike range banding, which would slice one object
        # in two wherever it crossed a band edge.
        total_pixels = float(width_px * height_px)
        min_pixels = int(min_obstacle_size_ratio * total_pixels)
        segment_mask = obstacle_mask & np.logical_not(getRangeEdgeMask(np_ranged, range_step_m))
        components = getComponents(segment_mask, min_pixels)

        ##############################
        # Build one entry per component
        timestamp = nepi_utils.get_time()
        last_cycle = PROCESS_DATA_DICT['last_cycle'].get(source_topic, None)
        match_dist = TRACK_MATCH_RATIO * math.sqrt(float(width_px) ** 2 + float(height_px) ** 2)
        delta_t = 0.0
        if last_cycle is not None:
            delta_t = timestamp - last_cycle['time']

        cycle_obstacles = []
        for comp in components[:max_obstacles]:
            obstacle_dict = buildObstacleDict(comp,
                                              np_ranged,
                                              segment_mask,
                                              az_deg_map,
                                              el_deg_map,
                                              total_pixels,
                                              timestamp)
            [uid, vel_pixels] = matchPrevious(obstacle_dict, last_cycle, match_dist, delta_t)
            obstacle_dict['uid'] = uid
            obstacle_dict['vel_pixels'] = vel_pixels
            obstacle_dict['name'] = getObstacleName(obstacle_dict)
            obstacles_dict_list.append(obstacle_dict)
            cycle_obstacles.append({'uid': uid,
                                    'cx': obstacle_dict['center_x'],
                                    'cy': obstacle_dict['center_y']})

        for i, obstacle_dict in enumerate(obstacles_dict_list):
            obstacle_dict['id'] = i

        recordCycle(source_topic, cycle_obstacles, timestamp)

    except Exception as e:
        logger.log_warn("Failed to process depth map: " + str(e), throttle_s = 5.0)
        return [], None, None, data_dict

    data_dict['data_time'] = start_time
    data_dict['process_time'] = nepi_utils.get_time() - start_time

    return obstacles_dict_list, depth_map_ground, depth_map_obstacles, data_dict

#########################
# Process Helpers
#########################




def getControlValue(controls_dict, control_name, default_value = None):
    # nepi_controls.get_control_value returns None for a control that is not in
    # the dict -- including one that create_controls_dict silently dropped for an
    # out-of-bounds default. Every caller here needs a usable number, so the
    # default stands in rather than propagating a None into the geometry.
    value = None
    try:
        value = nepi_controls.get_control_value(controls_dict, control_name)
    except Exception:
        value = None
    if value is None:
        value = default_value
    return value


def getNoReturnRange(np_ranged, set_no_return_range_m):
    # The range at and above which a return is a non-return, in metres, or None
    # when this frame has no such range. A positive control value is taken as
    # given; zero asks for detection.
    #
    # Detection tests the frame's maximum and nothing else. A clamp is the
    # maximum by construction -- nothing can exceed it -- and it holds a share
    # of the frame no single real surface holds at one exact float. Testing the
    # busiest value anywhere in the frame instead does NOT separate the two:
    # measured, a near-field cluster reaches 10.7% of a frame against a clamp
    # floor of 14.7%, while the maximum specifically reaches 0.40% of a frame
    # once the clamp is taken out of it.
    try:
        set_no_return_range_m = float(set_no_return_range_m)
    except (TypeError, ValueError):
        set_no_return_range_m = 0.0

    if set_no_return_range_m > 0.0:
        return set_no_return_range_m * (1.0 - NO_RETURN_TOLERANCE_RATIO)

    # Reduced over the finite pixels via where= rather than with np.nanmax over
    # the whole frame. Two reasons, both failure modes rather than tuning: a
    # single +inf pixel is the frame maximum to nanmax and drops the plateau
    # detection for the whole frame, and nanmax on an all-NaN frame emits a
    # RuntimeWarning per cycle. where= skips both, and the bool mask it needs is
    # the only allocation -- no compacted copy of the finite pixels.
    with np.errstate(invalid = 'ignore'):
        finite_mask = np.isfinite(np_ranged)
        max_range_m = float(np.max(np_ranged, initial = 0.0, where = finite_mask))
        if max_range_m <= 0.0:
            return None
        plateau_pixels = int(np.count_nonzero(np_ranged == max_range_m))

    if float(plateau_pixels) < NO_RETURN_MIN_RATIO * float(np_ranged.size):
        return None
    return max_range_m * (1.0 - NO_RETURN_TOLERANCE_RATIO)


def getGeometry(width_px, height_px, width_deg, height_deg):
    # Pinhole angular mapping across the reported field of view. Azimuth is
    # positive to the right of the boresight, elevation positive above it.
    #
    # A camera's image plane is FLAT, so a pixel's distance from the centre is
    # proportional to the TANGENT of its bearing, not to the bearing:
    #
    #     angle = atan(2 * ratio * tan(fov / 2))      ratio in [-0.5, +0.5]
    #
    # Spreading the field of view linearly over the pixels instead -- which is
    # what this did -- is exact only at the centre and at the two edges, and
    # wrong everywhere between. It put every reported azimuth_deg and
    # elevation_deg out by up to 1.75 deg on a 70 deg field of view, and since
    # height is range * sin(elevation) it dragged computed heights toward zero
    # near the horizon, on the wrong side of the ground threshold.
    #
    # The bearing maps and the sensor-frame unit vectors built from them are a
    # property of the sensor geometry alone, so they are cached and handed back
    # by reference. Callers must treat every array in the returned dict as
    # read-only -- the bearing maps are broadcast views and are not writable.
    cache_key = (int(width_px), int(height_px), float(width_deg), float(height_deg))
    geometry_dict = GEOMETRY_CACHE.get(cache_key, None)
    if geometry_dict is not None:
        return geometry_dict

    x_ratio = (np.arange(width_px, dtype = np.float32) + 0.5) / float(width_px) - 0.5
    y_ratio = 0.5 - (np.arange(height_px, dtype = np.float32) + 0.5) / float(height_px)

    # Image-plane offsets in units of the focal length. These are the pinhole
    # model itself; everything below is read off them.
    x_tan = 2.0 * x_ratio * math.tan(math.radians(float(width_deg)) / 2.0)
    y_tan = 2.0 * y_ratio * math.tan(math.radians(float(height_deg)) / 2.0)

    az_deg = np.degrees(np.arctan(x_tan)).astype(np.float32)
    el_deg = np.degrees(np.arctan(y_tan)).astype(np.float32)

    # A row vector and a column vector broadcast to the full frame without
    # materializing it. np.tile allocated two full float32 rasters per cycle to
    # hold values that repeat down every column and across every row.
    az_deg_map = np.broadcast_to(az_deg, (height_px, width_px))
    el_deg_map = np.broadcast_to(el_deg.reshape(height_px, 1), (height_px, width_px))

    # Sensor-frame unit vector per pixel: forward, right, up. Taken from the
    # image-plane offsets directly as (1, x_tan, y_tan) normalised, NOT rebuilt
    # from the two bearings above.
    #
    # Building it from the bearings would make the up-component sin(el), which
    # drops the azimuth term out of the normalisation -- exact down the centre
    # column and increasingly wrong across the frame. At the corner of a
    # 63.1 x 49.5 deg field of view that is 0.4187 against a true 0.3658, a 14%
    # height overstatement exactly where the ground threshold is being applied.
    # The cost is that the up-component is a real raster rather than a
    # broadcast view; it is built once per geometry and cached, not per cycle.
    x_tan_row = x_tan.reshape(1, width_px).astype(np.float32)
    y_tan_col = y_tan.reshape(height_px, 1).astype(np.float32)
    inv_norm = (1.0 / np.sqrt(1.0 + x_tan_row * x_tan_row + y_tan_col * y_tan_col)).astype(np.float32)

    geometry_dict = {
        'az_deg_map': az_deg_map,
        'el_deg_map': el_deg_map,
        'x_f': inv_norm,
        'y_f': x_tan_row * inv_norm,
        'z_f': y_tan_col * inv_norm,
    }

    if len(GEOMETRY_CACHE) >= MAX_GEOMETRY_CACHE_LEN:
        GEOMETRY_CACHE.clear()
    GEOMETRY_CACHE[cache_key] = geometry_dict
    return geometry_dict


def getLevelAngles(navpose_dict, use_navpose, mount_roll_deg, mount_pitch_deg):
    # Roll and pitch to level the height calculation with, as the SUM of two
    # independent terms. Precedence is additive, not either/or:
    #
    #   mount_*   the sensor's attitude in the frame the NavPose describes.
    #             Static, operator-declared, always applied.
    #   NavPose   that frame's attitude against gravity. Live, applied only
    #             when use_navpose is on and the source actually has one.
    #
    # With no NavPose the frame the mount angles are measured in IS level
    # ground, so they stand alone and a fixed downward-looking sensor is
    # finally describable -- which it was not, because this function returned
    # a flat [0.0, 0.0] for every file-published source and every source
    # without a NavPose, silently asserting a perfectly level camera.
    #
    # An operator whose NavPose is published for the sensor frame already,
    # rather than for a vehicle frame the sensor is bolted to, leaves the mount
    # angles at zero -- adding them there would count the mount twice.
    try:
        roll_deg = float(mount_roll_deg)
    except (TypeError, ValueError):
        roll_deg = 0.0
    try:
        pitch_deg = float(mount_pitch_deg)
    except (TypeError, ValueError):
        pitch_deg = 0.0

    if use_navpose != True or navpose_dict is None:
        return [roll_deg, pitch_deg]
    if navpose_dict.get('has_orientation', False) != True:
        return [roll_deg, pitch_deg]
    try:
        roll_deg += float(navpose_dict.get('roll_deg', 0.0))
        pitch_deg += float(navpose_dict.get('pitch_deg', 0.0))
    except Exception:
        pass
    return [roll_deg, pitch_deg]


def getHeightMap(np_ranged, geometry_dict, roll_deg, pitch_deg):
    # Height above the sensor, in the sensor's gravity-levelled frame. The
    # sensor-frame unit vector for a pixel comes from the cached geometry, and
    # is rotated by -roll about the boresight and -pitch about the horizontal
    # axis so the result is measured against gravity rather than against the
    # sensor housing.
    #
    # Only roll and pitch change between cycles, and both are scalars, so the
    # rotation collapses to three scalar weights over three cached rasters:
    #   z_l = z_f*cos(roll)*cos(pitch) - y_f*sin(roll)*cos(pitch) + x_f*sin(pitch)
    #
    # Sign convention, which the mount controls in PROCESS_CONTROLS_DICT follow:
    # at az 0 and roll 0 this reduces to sin(el + pitch), so a POSITIVE pitch
    # raises every pixel's world elevation -- nose up. Positive roll lowers the
    # right of the frame -- right side down. Confirmed against the data: a
    # source fitted at 5.0 deg nose down needs mount_pitch_deg -5.0 to read a
    # flat floor flat.
    #
    # UNVERIFIED: NavPose declares its roll_deg and pitch_deg in a right-handed
    # ENU frame with x forward, y left, z up, in which a right-handed rotation
    # about +y puts the nose DOWN for a positive pitch -- the opposite of what
    # this function assumes. If that reading is right, the NavPose path has the
    # pitch sign inverted and has since it was written. It is left alone here
    # because no NavPose-carrying capture was available to test it against and
    # flipping it blind would break any deployment that has been tuned around
    # it. Needs one level-then-tilted capture on hardware to settle.
    z_f = geometry_dict['z_f']

    roll_rad = math.radians(roll_deg)
    pitch_rad = math.radians(pitch_deg)
    sin_roll = math.sin(roll_rad)
    sin_pitch = math.sin(pitch_rad)

    if sin_roll == 0.0 and sin_pitch == 0.0:
        # Level, or no orientation to level with. The unrotated up-component is
        # already the answer, which is the path every source without a NavPose
        # takes on every cycle.
        return np_ranged * z_f

    cos_pitch = math.cos(pitch_rad)
    z_l = z_f * (math.cos(roll_rad) * cos_pitch)
    z_l -= geometry_dict['y_f'] * (sin_roll * cos_pitch)
    z_l += geometry_dict['x_f'] * sin_pitch

    return np_ranged * z_l


def getRangeEdgeMask(np_ranged, range_step_m):
    # A pixel is a range edge when ANY 8-neighbour differs from it by more than
    # range_step_m. Eight and not four, because getComponents labels with
    # connectivity 8: a diagonal pair the edge test never looked at is a pair
    # the labeller happily joins, so the one relationship that breaks
    # connectivity was blind to a quarter of the connections it had to break.
    # Measured on frame 788TzUTC, exactly ONE such pair -- 4.62 m against
    # 1.72 m at row 61, a 2.90 m step -- fused a 9168 px pillar into a
    # 110757 px wall and cost the pillar its own detection.
    #
    # NaN comparisons are False, so non-returns still never mark an edge --
    # they are already out of the mask.
    edge = np.zeros(np_ranged.shape, dtype = bool)
    with np.errstate(invalid = 'ignore'):
        dx = np.abs(np.diff(np_ranged, axis = 1)) > range_step_m
        dy = np.abs(np.diff(np_ranged, axis = 0)) > range_step_m
        # np.diff cannot express a diagonal, so these two are sliced by hand.
        # Same shape as the axis diffs and the same cost per pass.
        dd = np.abs(np_ranged[:-1, :-1] - np_ranged[1:, 1:]) > range_step_m
        da = np.abs(np_ranged[:-1, 1:] - np_ranged[1:, :-1]) > range_step_m
    # In place: each np.logical_or built a full temporary before the slice
    # assignment copied it straight back over the same view.
    edge[:, :-1] |= dx
    edge[:, 1:] |= dx
    edge[:-1, :] |= dy
    edge[1:, :] |= dy
    edge[:-1, :-1] |= dd
    edge[1:, 1:] |= dd
    edge[:-1, 1:] |= da
    edge[1:, :-1] |= da
    return edge


def getComponents(obstacle_mask, min_pixels):
    mask_u8 = obstacle_mask.astype(np.uint8)
    [count, labels, stats, centroids] = cv2.connectedComponentsWithStats(mask_u8, connectivity = 8)
    components = []
    # Label 0 is the background of the mask, never an obstacle.
    for label in range(1, count):
        area_pixels = int(stats[label, cv2.CC_STAT_AREA])
        if area_pixels < min_pixels or area_pixels <= 0:
            continue
        components.append({
            'label': label,
            'labels': labels,
            'xmin': int(stats[label, cv2.CC_STAT_LEFT]),
            'ymin': int(stats[label, cv2.CC_STAT_TOP]),
            'width': int(stats[label, cv2.CC_STAT_WIDTH]),
            'height': int(stats[label, cv2.CC_STAT_HEIGHT]),
            'area_pixels': area_pixels,
            'cx': float(centroids[label][0]),
            'cy': float(centroids[label][1]),
        })
    components.sort(key = lambda c: c['area_pixels'], reverse = True)
    return components


def buildObstacleDict(comp, np_ranged, obstacle_mask, az_deg_map, el_deg_map, total_pixels, timestamp):
    # Every member of a component lies inside its own bounding box, so the
    # membership test and all three reductions run on that box rather than on
    # the full frame. With max_obstacles at 10 the old form made ten
    # full-frame comparisons and forty full-frame gathers per cycle.
    x0 = comp['xmin']
    y0 = comp['ymin']
    x1 = x0 + comp['width']
    y1 = y0 + comp['height']

    member_mask = (comp['labels'][y0:y1, x0:x1] == comp['label']) & obstacle_mask[y0:y1, x0:x1]
    has_members = member_mask.any()

    ranges = np_ranged[y0:y1, x0:x1][member_mask]
    ranges = ranges[np.isfinite(ranges)]
    if ranges.size > 0:
        # Median rather than mean: a depth map edge pixel that straddles the
        # obstacle and the background behind it reads as a far outlier, and the
        # median is what keeps one such pixel from pushing the reported range
        # past the obstacle.
        range_m = float(np.median(ranges))
    else:
        range_m = FLOAT_FIELD_UNSET

    azimuth_deg = float(np.mean(az_deg_map[y0:y1, x0:x1][member_mask])) if has_members else FLOAT_FIELD_UNSET
    elevation_deg = float(np.mean(el_deg_map[y0:y1, x0:x1][member_mask])) if has_members else FLOAT_FIELD_UNSET

    xmin = x0
    ymin = y0
    width_pixels = comp['width']
    height_pixels = comp['height']

    return {
        'timestamp': float(timestamp),
        'name': '',
        'id': 0,
        'uid': '',
        # Fraction of the bounding box actually occupied by obstacle returns.
        # A solid box reads 1.0; a sparse or ragged group reads low.
        'confidence': float(comp['area_pixels']) / float(max(width_pixels * height_pixels, 1)),

        'xmin_pixel': xmin,
        'ymin_pixel': ymin,
        'xmax_pixel': xmin + width_pixels,
        'ymax_pixel': ymin + height_pixels,
        'width_pixels': width_pixels,
        'height_pixels': height_pixels,
        'area_pixels': float(comp['area_pixels']),
        'area_ratio': float(comp['area_pixels']) / float(max(total_pixels, 1.0)),
        'vel_pixels': [0.0, 0.0, 0.0],

        'range_m': range_m,
        'azimuth_deg': azimuth_deg,
        'elevation_deg': elevation_deg,

        # Not an Obstacle.msg field; carried for tracking and dropped by the
        # caller, which only reads the fields it needs.
        'center_x': comp['cx'],
        'center_y': comp['cy'],
    }


def matchPrevious(obstacle_dict, last_cycle, match_dist, delta_t):
    cx = obstacle_dict['center_x']
    cy = obstacle_dict['center_y']

    best = None
    best_dist = match_dist
    if last_cycle is not None:
        for prev in last_cycle['obstacles']:
            dist = math.sqrt((cx - prev['cx']) ** 2 + (cy - prev['cy']) ** 2)
            if dist < best_dist:
                best_dist = dist
                best = prev

    if best is None:
        uid = 'obstacle_' + str(PROCESS_DATA_DICT['next_id'])
        PROCESS_DATA_DICT['next_id'] = PROCESS_DATA_DICT['next_id'] + 1
        return [uid, [0.0, 0.0, 0.0]]

    if delta_t > 0.001:
        vel_pixels = [(cx - best['cx']) / delta_t, (cy - best['cy']) / delta_t, 0.0]
    else:
        vel_pixels = [0.0, 0.0, 0.0]
    return [best['uid'], vel_pixels]


def getObstacleName(obstacle_dict):
    # An obstacle is a shape in a range image, not a class, so every obstacle
    # carries the same name.
    return 'obstacle'


def recordCycle(source_topic, cycle_obstacles, timestamp):
    PROCESS_DATA_DICT['last_cycle'][source_topic] = {
        'time': timestamp,
        'obstacles': cycle_obstacles,
    }



def init_process_controls_dict():
    """Return a copy of this module's control definition dictionary.

    Returns:
        dict: The nepi_controls init-dict form of every control this module's
            process function reads. Callers hand this to a ControlsIF.
    """
    return copy.deepcopy(PROCESS_CONTROLS_DICT)


def init_process_data_dict():
    """Return a copy of this module's per-cycle process data dictionary.

    Returns:
        dict: The PROCESS_DATA_DICT bookkeeping form this module's process
            function reads and writes. Callers hand this to an ObstaclesIF.
    """
    return copy.deepcopy(PROCESS_DATA_DICT)
