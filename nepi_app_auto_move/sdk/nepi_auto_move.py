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

from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_auto_move"
logger = Logger(log_name = log_name)


########################
## Library Data

# Control definitions in the nepi_controls init-dict form. A node deep-copies
# this and hands it to a ControlsIF, which turns it into a live controls dict
# and passes that dict back to plan_move on every goto.
#
# Every default must sit inside its own bounds. nepi_controls.create_controls_dict
# raises NameError on an out-of-bounds Float default and the bare except around
# the per-control body silently DROPS the control, so an out-of-bounds default
# does not clamp -- it makes the control disappear.
#
# These are the controls a real planner needs. The placeholder planner below
# reads only max_step_m; the rest are declared now so the operator-facing
# surface does not move when the planner is filled in.
MOVE_CONTROLS_DICT = {

    'max_step_m': {
        'type': 'Float', 'default': 2.0, 'bounds': [0.1, 50.0], 'round_value': 2,
        'display_name': 'Max Step (m)',
        'description': 'Longest single move the plan may contain. A longer '
                       'request is split into several steps.',
        'hidden': False},

    'standoff_m': {
        'type': 'Float', 'default': 0.5, 'bounds': [0.0, 20.0], 'round_value': 2,
        'display_name': 'Standoff (m)',
        'description': 'Stop this far short of the clicked point.',
        'hidden': False},

    'clearance_m': {
        'type': 'Float', 'default': 0.5, 'bounds': [0.0, 20.0], 'round_value': 2,
        'display_name': 'Obstacle Clearance (m)',
        'description': 'Closest the planned path may pass to a known obstacle.',
        'hidden': False},

    'hold_altitude': {
        'type': 'Bool', 'default': True,
        'display_name': 'Hold Altitude',
        'description': 'Keep the current height, ignoring the vertical part of '
                       'the clicked offset.',
        'hidden': False},

    'face_target': {
        'type': 'Bool', 'default': False,
        'display_name': 'Face Target',
        'description': 'Turn to face the clicked bearing before moving.',
        'hidden': False},

    'avoid_obstacles': {
        'type': 'Bool', 'default': False,
        'display_name': 'Avoid Obstacles',
        'description': 'Plan around known obstacles instead of moving straight '
                       'to the clicked point.',
        'hidden': False},

    }


#########################
# Move Planning
#########################

##############################################################################
# EXTENSION POINT
#
# plan_move() below is the placeholder. It is the ONLY thing a developer adding
# real move logic has to replace.
#
# WHAT TO CHANGE
#   The body of plan_move(). Read the inputs, return a list of move steps.
#   Everything a planner needs is already handed in -- the requested offset, the
#   current depth map, the object and target lists, the robot's reported state,
#   and the live control values. A planner that needs something not on that list
#   is a signal to widen this signature deliberately, not to reach for ROS from
#   inside this module.
#
# WHAT NOT TO CHANGE
#   The signature and the return shape. AutoMoveIF.runPlanning() calls this with
#   exactly these six arguments and hands each returned step straight to
#   ConnectRBXDeviceIF.goto_position(). Changing either without changing that
#   caller breaks the goto process silently -- a step with an unexpected key
#   simply moves zero on that axis.
#
#   Do not import ROS, nepi_sdk transport, or any interface class here. This
#   module is a pure function of its arguments so a test can call it with
#   fixtures and any other node can call it with the same six arguments.
#
#   Do not block. plan_move() runs on the goto process timer thread; a planner
#   that needs to iterate should return the steps it has and let the process
#   call it again on the next goto.
#
# UNITS
#   goto_dict, and every value in every returned step, are METRES and DEGREES.
#   np_depth_map is MILLIMETRES -- that is the platform contract for a depth map
#   array, and the *_range_m bounds on its status message are metres. Divide by
#   1000.0 at the point of use and comment the conversion; mixing the two
#   produces a plan that looks reasonable and is a thousand times wrong.
#
#   The frame is the robot body frame from nepi_interfaces/GotoPosition:
#   x forward, y LEFT, z up, yaw positive to port, all relative to where the
#   robot is when the step is issued -- steps compose, they are not absolute.
#
# STATUS FIELDS THE PROCESS UPDATES
#   The process owns every status field; a planner writes none of them. It sets
#   goto_state through idle -> planning -> moving -> complete (or cancelled),
#   and goto_step / goto_step_count from the length of the list returned here,
#   advancing one step per completed move. A step's 'description' is copied into
#   goto_msg while that step runs, so it is worth writing for an operator.
#
#   Returning an empty list is a legitimate answer -- "no move needed". The
#   process reports COMPLETE for it rather than treating it as a failure.
##############################################################################

def plan_move(goto_dict, np_depth_map, objects_list, targets_list, robot_dict, controls_dict):
    """Plan the sequence of relative moves that satisfies a goto request.

    This is the placeholder implementation. It returns a single direct move to
    the requested offset with no avoidance of any kind: the depth map, the
    object list, the target list and the robot state are accepted and ignored.
    Replace the body to add real move logic; see the EXTENSION POINT block above
    this function for the contract it must keep.

    Args:
        goto_dict (dict): The requested relative move, in METERS, in the robot
            body frame (x forward, y left, z up). Keys: 'x_m', 'y_m', 'z_m',
            and 'max_move_m', the largest forward distance the operator allows.
        np_depth_map (numpy.ndarray): The newest depth map frame for the
            selected image source, in MILLIMETERS, or None when no depth map is
            connected. Non-returns are NaN or non-positive.
        objects_list (list): Detected objects for the selected image, each a
            dict converted from a nepi_interfaces/Detection. Pixel bounds are
            'xmin'/'ymin'/'xmax'/'ymax'; 'range_m' is METERS. Empty when no
            detector is running on this image.
        targets_list (list): Located targets for the selected image, each a dict
            converted from a nepi_interfaces/Target. Pixel bounds carry the
            '_pixel' suffix; 'range_m' is METERS. Empty when none are published.
        robot_dict (dict): What the RBX connect interface reports about the
            selected robot. Keys: 'namespace', 'connected', 'ready', and
            'status_dict', the DeviceRBXStatus as a dict or None.
        controls_dict (dict): The live nepi_controls dictionary for this app's
            control set, built from MOVE_CONTROLS_DICT above.

    Returns:
        list: Move steps, executed in order, each a dict with keys 'x_m',
            'y_m', 'z_m' (METERS, relative, body frame), 'yaw_deg' (DEGREES,
            relative, zero holds the current heading) and 'description', a short
            operator-facing string. An empty list means no move is needed.
    """
    x_m = float(goto_dict.get('x_m', 0.0))
    y_m = float(goto_dict.get('y_m', 0.0))
    z_m = float(goto_dict.get('z_m', 0.0))

    # PLACEHOLDER: one straight move to the requested offset. Nothing here
    # consults np_depth_map, objects_list, targets_list or robot_dict -- a real
    # planner is exactly the code that does.
    distance_m = math.sqrt((x_m * x_m) + (y_m * y_m) + (z_m * z_m))
    if distance_m <= 0.0:
        logger.log_info("Goto request is a zero move, no steps planned")
        return []

    step_dict = {
        'x_m': x_m,
        'y_m': y_m,
        'z_m': z_m,
        # Zero holds the current heading, per the GotoPosition contract. A
        # planner that turns the robot sets this.
        'yaw_deg': 0.0,
        'description': 'Direct move ' + str(round(distance_m, 2)) + 'm',
    }

    return [step_dict]


def get_controls_dict():
    """Return a copy of this module's control definition dictionary.

    Returns:
        dict: The nepi_controls init-dict form of MOVE_CONTROLS_DICT, safe for
            a caller to hand to a ControlsIF and modify.
    """
    return copy.deepcopy(MOVE_CONTROLS_DICT)
