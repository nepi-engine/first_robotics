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

    'ground_max_height_m': {
        'type': 'Float', 'default': -0.3, 'bounds': [-10.0, 10.0], 'round_value': 2,
        'display_name': 'Ground Height (m)',
        'description': 'Returns at or below this height relative to the sensor are ground, not obstacles.',
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


#########################
# Process Functions
#########################





def process_data(np_depth_map, status_dict, navpose_dict, data_dict, controls_dict):
    """Segment a depth map into ground and obstacles and locate each obstacle.

    Range-gates the depth map, converts every remaining return to a height in a
    gravity-levelled sensor frame, splits the returns into ground and obstacle
    sets on a height threshold, groups the obstacle returns into connected
    components, and reports one entry per component with its pixel box, range,
    azimuth, elevation, and pixel velocity.

    Args:
        np_depth_map (numpy.ndarray): Range image in metres, shape (H, W).
            Non-returns may be NaN, inf, or zero.
        status_dict (dict): The source's DepthMapStatus as a dictionary. Read
            for 'width_deg', 'height_deg' and 'depth_map_topic'.
        navpose_dict (dict): The source's NavPose as a dictionary. Read for
            'has_orientation', 'roll_deg' and 'pitch_deg'.
        controls_dict (dict): Live nepi_controls controls dictionary built from
            PROCESS_CONTROLS_DICT.

    Returns:
        list: obstacles_dict_list -- one dict per obstacle, ordered largest
            first, carrying every field of nepi_app_obstacles/Obstacle.
        numpy.ndarray: depth_map_ground -- the range-gated depth map with every
            non-ground pixel set to NaN, or None if no ground was found.
        numpy.ndarray: depth_map_obstacles -- the range-gated depth map with
            every non-obstacle pixel set to NaN, or None if no obstacle
            returns were found.
    """
    obstacles_dict_list = []
    depth_map_ground = None
    depth_map_obstacles = None

    start_time = nepi_utils.get_time()

    if np_depth_map is None:
        return obstacles_dict_list, depth_map_ground, depth_map_obstacles

    try:
        np_depth_map = np.asarray(np_depth_map, dtype = np.float32)
        if np_depth_map.ndim != 2:
            logger.log_warn("Depth map is not a single channel range image: " + str(np_depth_map.shape), throttle_s = 5.0)
            return obstacles_dict_list, depth_map_ground, depth_map_obstacles

        min_range_m = getControlValue(controls_dict, 'min_range_m', 0.5)
        max_range_m = getControlValue(controls_dict, 'max_range_m', 10.0)
        ground_max_height_m = getControlValue(controls_dict, 'ground_max_height_m', -0.3)
        range_step_m = getControlValue(controls_dict, 'range_step_m', 0.5)
        min_obstacle_size_ratio = getControlValue(controls_dict, 'min_obstacle_size_ratio', 0.02)
        max_obstacles = int(getControlValue(controls_dict, 'max_obstacles', 10))
        use_navpose = getControlValue(controls_dict, 'use_navpose', True)

        if max_range_m <= min_range_m:
            max_range_m = min_range_m + 0.1

        height_px, width_px = np_depth_map.shape
        width_deg = float(status_dict.get('width_deg', DEFAULT_WIDTH_DEG) or DEFAULT_WIDTH_DEG)
        height_deg = float(status_dict.get('height_deg', DEFAULT_HEIGHT_DEG) or DEFAULT_HEIGHT_DEG)
        source_topic = str(status_dict.get('depth_map_topic', '') or '')

        ##############################
        # Range gate
        np_ranged = copy.deepcopy(np_depth_map)
        np_ranged[np.isinf(np_ranged)] = np.nan
        np_ranged[np_ranged <= 0] = np.nan
        np_ranged[np_ranged < min_range_m] = np.nan
        np_ranged[np_ranged > max_range_m] = np.nan

        valid_mask = np.isfinite(np_ranged)
        if valid_mask.any() == False:
            return obstacles_dict_list, depth_map_ground, depth_map_obstacles

        ##############################
        # Per-pixel bearing and height
        [az_deg_map, el_deg_map] = getBearingMaps(width_px, height_px, width_deg, height_deg)
        [roll_deg, pitch_deg] = getLevelAngles(navpose_dict, use_navpose)
        np_height = getHeightMap(np_ranged, az_deg_map, el_deg_map, roll_deg, pitch_deg)

        ##############################
        # Ground / obstacle split
        ground_mask = valid_mask & (np_height <= ground_max_height_m)
        obstacle_mask = valid_mask & (np_height > ground_max_height_m)

        if ground_mask.any():
            depth_map_ground = np.full(np_ranged.shape, np.nan, dtype = np.float32)
            depth_map_ground[ground_mask] = np_ranged[ground_mask]
        if obstacle_mask.any():
            depth_map_obstacles = np.full(np_ranged.shape, np.nan, dtype = np.float32)
            depth_map_obstacles[obstacle_mask] = np_ranged[obstacle_mask]
        else:
            recordCycle(source_topic, [], start_time)
            return obstacles_dict_list, depth_map_ground, depth_map_obstacles

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
            [obstacle_dict, member_mask] = buildObstacleDict(comp,
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
        return [], None, None

    data_dict['data_time'] = start_time
    data_dict['process_time'] = nepi_utils.get_time() - start_time

    return obstacles_dict_list, depth_map_ground, depth_map_obstacles, data_dict

#########################
# Process Helpers
#########################




def getControlValue(controls_dict, control_name):
    try:
        value = nepi_controls.get_control_value(controls_dict, control_name)
    except Exception:
        value = None
    return value


def getBearingMaps(width_px, height_px, width_deg, height_deg):
    # Linear angular mapping across the reported field of view. Azimuth is
    # positive to the right of the boresight, elevation positive above it.
    x_ratio = (np.arange(width_px, dtype = np.float32) + 0.5) / float(width_px) - 0.5
    y_ratio = 0.5 - (np.arange(height_px, dtype = np.float32) + 0.5) / float(height_px)
    az_deg = x_ratio * float(width_deg)
    el_deg = y_ratio * float(height_deg)
    az_deg_map = np.tile(az_deg, (height_px, 1))
    el_deg_map = np.tile(el_deg.reshape(height_px, 1), (1, width_px))
    return [az_deg_map, el_deg_map]


def getLevelAngles(navpose_dict, use_navpose):
    if use_navpose != True or navpose_dict is None:
        return [0.0, 0.0]
    if navpose_dict.get('has_orientation', False) != True:
        return [0.0, 0.0]
    try:
        return [float(navpose_dict.get('roll_deg', 0.0)), float(navpose_dict.get('pitch_deg', 0.0))]
    except Exception:
        return [0.0, 0.0]


def getHeightMap(np_ranged, az_deg_map, el_deg_map, roll_deg, pitch_deg):
    # Height above the sensor, in the sensor's gravity-levelled frame. The
    # sensor-frame unit vector for a pixel is built from its azimuth and
    # elevation, then rotated by -roll about the boresight and -pitch about the
    # horizontal axis so the result is measured against gravity rather than
    # against the sensor housing.
    az_rad = np.radians(az_deg_map)
    el_rad = np.radians(el_deg_map)

    x_f = np.cos(el_rad) * np.cos(az_rad)   # forward
    y_f = np.cos(el_rad) * np.sin(az_rad)   # right
    z_f = np.sin(el_rad)                    # up

    roll_rad = math.radians(roll_deg)
    pitch_rad = math.radians(pitch_deg)

    # Un-roll about the forward axis
    z_r = z_f * math.cos(roll_rad) - y_f * math.sin(roll_rad)
    # Un-pitch about the right axis
    z_l = z_r * math.cos(pitch_rad) + x_f * math.sin(pitch_rad)

    return np_ranged * z_l


def getRangeEdgeMask(np_ranged, range_step_m):
    # A pixel is a range edge when either 4-neighbour in the +x or +y direction
    # differs from it by more than range_step_m. NaN comparisons are False, so
    # non-returns never mark an edge -- they are already out of the mask.
    edge = np.zeros(np_ranged.shape, dtype = bool)
    with np.errstate(invalid = 'ignore'):
        dx = np.abs(np.diff(np_ranged, axis = 1)) > range_step_m
        dy = np.abs(np.diff(np_ranged, axis = 0)) > range_step_m
    edge[:, :-1] = np.logical_or(edge[:, :-1], dx)
    edge[:, 1:] = np.logical_or(edge[:, 1:], dx)
    edge[:-1, :] = np.logical_or(edge[:-1, :], dy)
    edge[1:, :] = np.logical_or(edge[1:, :], dy)
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
    member_mask = (comp['labels'] == comp['label']) & obstacle_mask

    ranges = np_ranged[member_mask]
    ranges = ranges[np.isfinite(ranges)]
    if ranges.size > 0:
        # Median rather than mean: a depth map edge pixel that straddles the
        # obstacle and the background behind it reads as a far outlier, and the
        # median is what keeps one such pixel from pushing the reported range
        # past the obstacle.
        range_m = float(np.median(ranges))
    else:
        range_m = FLOAT_FIELD_UNSET

    azimuth_deg = float(np.mean(az_deg_map[member_mask])) if member_mask.any() else FLOAT_FIELD_UNSET
    elevation_deg = float(np.mean(el_deg_map[member_mask])) if member_mask.any() else FLOAT_FIELD_UNSET

    xmin = comp['xmin']
    ymin = comp['ymin']
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
    }, member_mask


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
    """Return a copy of this module's control definition dictionary.

    Returns:
        dict: The nepi_controls init-dict form of every control this module's
            process function reads. Callers hand this to a ControlsIF.
    """
    return copy.deepcopy(PROCESS_CONTROLS_DICT)
