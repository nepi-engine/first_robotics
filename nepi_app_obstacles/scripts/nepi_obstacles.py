#!/usr/bin/env python
##
## Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
##
## This file is part of nepi-engine
## (see https://github.com/nepi-engine).
##
## License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
##

"""Obstacle 'process' module -- control definitions and per-cycle process functions.

Same registry shape as nepi_sdk/nepi_stab_pt.py and the sibling
nepi_app_stereo_cam/scripts/stereo_settings.py:

  * PROCESSES_DICT        -- registry of named processes, each with a
                             process_function + default_controls_dict.
  * DEFAULT_PROCESS       -- the process the node selects before an operator picks.
  * get_blank_data_dict() -- fresh runtime data dict for one process call.

What differs from those two siblings, and the reason this module exists in this
shape: their default_settings_dict is a hand-rolled dict of raw values that the
node had to sanitize itself (create_processes_dict / update_processes_dict), with
a nested *_controls_dict the RUI populated by convention. Here each
default_controls_dict is a nepi_controls CONTROLS INIT DICT instead -- the
declarative form nepi_sdk/nepi_controls.create_controls_dict() consumes -- and
obstacles_app_node.py hands each set to its own ControlsIF. The IF then owns
type/bounds/options validation, RUI presentation, and param persistence, which is
why the hand-rolled create_processes_dict / update_processes_dict pair is gone
rather than ported: nepi_controls does that work now. See the module note at the
bottom for what each removed function was replaced by.

Init dict key schema, per nepi_controls.create_controls_dict():
  'type'      REQUIRED, one of nepi_controls.CONTROL_TYPES. An entry whose type
              is missing or unrecognized is silently dropped.
  'default'   REQUIRED by every type except Trigger.
  'options'   REQUIRED by Menu / Selection / Selections.
  'bounds'    [min, max] for Int / Float / FloatSlider. -999 means no limit.
  Any remaining key is copied through only if it is also a nepi_interfaces/Control
  field: 'display_name', 'description', 'hidden', 'round_value', 'round_display'.
  ('value_round' / 'display_round', as spelled in nepi_controls.EXAMPLE_INIT_DICT,
  are NOT Control fields and are ignored.)

Two nepi_controls types are deliberately unused in both sets below:
  FloatSliders -- create_controls_dict()'s FloatSliders branch references an
                  undefined name `value` (nepi_controls.py, `if value[0] >
                  value[1]`), the surrounding except swallows the NameError, and
                  the control is dropped. A range window would be the natural
                  widget for min/max range, so it is written as two Float
                  controls until that branch is fixed.
  Trigger      -- Nepi_IF_Controls.js publishes UpdateString to
                  set_trigger_control_value, which ControlsIF subscribes to as
                  UpdateTrigger, so a Trigger control cannot be fired from the
                  RUI today. Nothing here depends on one.

Each process_function(process_data_dict, process_controls_dict) is called once per
processing cycle by obstacles_app_node.py. process_controls_dict is a flat
{control_name: current_value} snapshot the node reads out of that process's
ControlsIF, so a control is read here by exactly the key it is authored under
below. The signature is unchanged from the pre-migration stubs.
"""


import copy

from nepi_interfaces.msg import Targets

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_nav



from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_process"
logger = Logger(log_name = log_name)


SOURCE_MESSAGE_DICT = {'Targets' : Targets}

DATA_DICT = {
    # Required Fields
    'data_time': 0.0,
    'process_time': 0.0,

    # Inputs, as supplied by the node once per cycle. Each is the cached data
    # dict of one of this app's connect IFs, stored VERBATIM rather than unpacked
    # into named fields -- the node caches whatever ConnectDepthMapIF /
    # ConnectTargetsIF / ConnectNavPoseIF hand their dataCB, and inventing key
    # names for the insides of those dicts here would be guessing at another IF's
    # contract. None means that source is unselected or has not produced data.
    'depth_map_dict': None,
    'targets_dict': None,
    'navpose_dict': None,

    # Outputs. obstacles is a list of OBSTACLE_DICT-shaped dicts, one per
    # obstacle, NOT a list of nepi_app_obstacles/Obstacle messages. This module
    # is imported by obstacles_app_node.py and re-imported by its
    # reload_processes path, and it must stay importable in contexts where the
    # generated message types are not on the path, so the message conversion
    # lives in the node (convertObstaclesMsg()) and this side stays plain dicts.
    'obstacles': [],
    'obstacle_count': 0,
}


# Unset-field constants, duplicated from the constants of the same names declared
# in msg/Obstacle.msg.
#
# THEY MUST STAY IN SYNC WITH msg/Obstacle.msg. They are redeclared here rather
# than imported from nepi_app_obstacles.msg because this module has to stay
# importable where the generated messages are not available -- see the
# 'obstacles' note in DATA_DICT above -- and an import of the generated type here
# would make a process function unloadable in exactly the case the node's
# reload_processes path is meant to survive.
INT_FIELD_UNSET = -999
FLOAT_FIELD_UNSET = -999.0


# Per-obstacle dict schema.
#
# THE RULE: every key here corresponds to one field of msg/Obstacle.msg with the
# SAME NAME, and the two are edited together. Adding a field to Obstacle.msg
# without adding the matching key here means the node's convertObstaclesMsg()
# has nothing to read for it; adding a key here without the message field means
# the value is computed every cycle and silently dropped at conversion.
#
# Each default is the unset value for that field's type: numeric fields to the
# constants above, string fields to ''. A consumer reads those as 'no value this
# cycle' -- see docs/ADDING_OBSTACLE_MSG_FIELDS.md.
OBSTACLE_DICT = {
    'timestamp': FLOAT_FIELD_UNSET,
    'name': '',
    'id': INT_FIELD_UNSET,
    'uid': '',
    'confidence': FLOAT_FIELD_UNSET,
}


PROCESSES_DICT = dict()

DEFAULT_PROCESS = 'process_1'

#########################
# Process Process Functions
#########################




############################

# process_1 -- depth-map obstacle extraction.
#
# Every control below is answerable from what obstacles_app_node.py actually
# consumes: a depth map (ConnectDepthMapIF), the color image that belongs to it,
# and NavPose. Nothing here reads targets -- that is process_2's job -- so this
# set carries no detection-confidence control.
process_1_controls = {

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

    'min_obstacle_size_ratio': {
        'type': 'FloatSlider', 'default': 0.02, 'bounds': [0.0, 1.0], 'round_value': 3,
        'display_name': 'Min Obstacle Size',
        'description': 'Smallest obstacle to report, as a fraction of the depth map area.',
        'hidden': False},

    'use_navpose': {
        'type': 'Bool', 'default': True,
        'display_name': 'Use NavPose',
        'description': 'Report obstacle positions in the NavPose frame when a NavPose source is connected.',
        'hidden': False},

    }


def process_1(process_data_dict,
                process_controls_dict
                ):

    #logger.log_info("******")
    #logger.log_info("*** Processs Solution Update Starting ***")
    #logger.log_info("******")
    start_time = nepi_utils.get_time()

    # Current control values, read by the keys process_1_controls authors above.
    # The node snapshots these off this process's ControlsIF once per cycle, so a
    # missing key means the control was dropped at create_controls_dict() time
    # rather than that the operator changed something.
    min_range_m = process_controls_dict.get('min_range_m', 0.5)
    max_range_m = process_controls_dict.get('max_range_m', 10.0)
    min_obstacle_size_ratio = process_controls_dict.get('min_obstacle_size_ratio', 0.02)
    use_navpose = process_controls_dict.get('use_navpose', True)

    depth_map_dict = process_data_dict['depth_map_dict']
    navpose_dict = process_data_dict['navpose_dict']

    # process_1 emits NO obstacles, and that is a statement about the message
    # rather than about the depth map.
    #
    # This process consumes only the depth map and the NavPose. A depth return
    # is a geometry -- a range, a bearing, a size, a position -- and
    # msg/Obstacle.msg declares no geometry field today: it carries timestamp,
    # name, id, uid and confidence, which are identity and detection-quality
    # fields. There is no honest way to name, identify or score an obstacle from
    # a depth map alone, so emitting one here would mean inventing a name and a
    # confidence for it, and the obstacle's actual content -- where it is --
    # would have nowhere to go.
    #
    # This is the FIRST thing the next developer changes: add the geometry fields
    # to msg/Obstacle.msg and the matching keys to OBSTACLE_DICT, then fill them
    # here. Until then the depth map is read, the controls above are live and
    # operator-settable, and the emitted list is empty.
    obstacles = []
    if depth_map_dict is not None:
        pass

    process_data_dict['obstacles'] = obstacles
    process_data_dict['obstacle_count'] = len(obstacles)
    process_data_dict['data_time'] = start_time
    process_data_dict['process_time'] = nepi_utils.get_time() - start_time

    return process_data_dict, process_controls_dict



PROCESSES_DICT['process_1'] = {'process_function': process_1,
                                'default_controls_dict': process_1_controls}


############################


# process_2 -- targets-fused obstacle extraction.
#
# Same depth range gate as process_1, plus the two controls that only make sense
# once the app's Targets source is in play: a confidence floor on the detections
# it fuses (Target/Targets carry a confidence field) and a cap on how many
# obstacles one cycle may report.
process_2_controls = {

    'min_range_m': {
        'type': 'Float', 'default': 0.5, 'bounds': [0.1, 100.0], 'round_value': 2,
        'display_name': 'Min Range (m)',
        'description': 'Ignore depth map returns closer than this range.',
        'hidden': False},

    'max_range_m': {
        'type': 'Float', 'default': 20.0, 'bounds': [0.2, 100.0], 'round_value': 2,
        'display_name': 'Max Range (m)',
        'description': 'Ignore depth map returns farther than this range.',
        'hidden': False},

    'min_target_confidence': {
        'type': 'FloatSlider', 'default': 0.5, 'bounds': [0.0, 1.0], 'round_value': 2,
        'display_name': 'Min Target Confidence',
        'description': 'Ignore detected targets below this confidence when fusing them into obstacles.',
        'hidden': False},

    'max_obstacles': {
        'type': 'Int', 'default': 10, 'bounds': [1, 100],
        'display_name': 'Max Obstacles',
        'description': 'Most obstacles to report from one processing cycle.',
        'hidden': False},

    'use_navpose': {
        'type': 'Bool', 'default': True,
        'display_name': 'Use NavPose',
        'description': 'Report obstacle positions in the NavPose frame when a NavPose source is connected.',
        'hidden': False},

    }



def process_2(process_data_dict,
                process_controls_dict
                ):

    #logger.log_info("******")
    #logger.log_info("*** Processs Solution Update Starting ***")
    #logger.log_info("******")
    start_time = nepi_utils.get_time()

    # Current control values, read by the keys process_2_controls authors above.
    min_range_m = process_controls_dict.get('min_range_m', 0.5)
    max_range_m = process_controls_dict.get('max_range_m', 20.0)
    min_target_confidence = process_controls_dict.get('min_target_confidence', 0.5)
    max_obstacles = process_controls_dict.get('max_obstacles', 10)
    use_navpose = process_controls_dict.get('use_navpose', True)

    depth_map_dict = process_data_dict['depth_map_dict']
    targets_dict = process_data_dict['targets_dict']
    navpose_dict = process_data_dict['navpose_dict']

    # Every field msg/Obstacle.msg declares today -- timestamp, name, id, uid,
    # confidence -- is a property a detected target already carries, so this
    # process fills them for real rather than emitting an empty list the way
    # process_1 does.
    #
    # Source shape, from ConnectTargetsIF._dataCb (nepi_api/connect_targets_if.py):
    # the dict handed to dataCB has keys 'namespace', 'data', 'timestamp' and
    # 'process_time', where 'data' is the whole nepi_interfaces/Targets message
    # run through nepi_sdk.convert_msg2dict(). So the per-target list is at
    # targets_dict['data']['targets'], and each entry is a dict keyed by the
    # nepi_interfaces/Target field names.
    #
    # Only the targets source is read here. The depth map is NOT fused in --
    # combining a depth return with a target is the geometry work that has
    # nowhere to land until msg/Obstacle.msg gains geometry fields, the same
    # reason process_1 emits nothing.
    obstacles = []
    if targets_dict is not None:
        targets_data = targets_dict.get('data', None)
        target_list = []
        if targets_data is not None:
            target_list = targets_data.get('targets', []) or []

        for target in target_list:
            if len(obstacles) >= max_obstacles:
                # max_obstacles control: cap the emitted list. Discovery order is
                # the detector's own order; there is no better ranking to apply
                # while Obstacle.msg carries no geometry to rank on.
                break

            confidence = target.get('confidence', FLOAT_FIELD_UNSET)
            if confidence == FLOAT_FIELD_UNSET or confidence < min_target_confidence:
                continue

            # Range gate. nepi_interfaces/Target DOES carry range_m, so
            # min_range_m / max_range_m are live here rather than inert -- but
            # the producer fills it with FLOAT_FIELD_UNSET whenever it had no
            # depth map to measure against (nepi_api/node_if_ai_detector.py
            # leaves target_range_m at -999 when np_depth_map is None). Gating on
            # an unset range would silently drop every obstacle from any detector
            # running without depth, so an unset range skips the gate instead of
            # failing it.
            range_m = target.get('range_m', FLOAT_FIELD_UNSET)
            if range_m != FLOAT_FIELD_UNSET:
                if range_m < min_range_m or range_m > max_range_m:
                    continue

            obstacle = get_blank_obstacle_dict()
            obstacle['name'] = target.get('name', '')
            obstacle['uid'] = target.get('uid', '')
            obstacle['confidence'] = confidence
            # Epoch seconds of the source image the target was detected in, which
            # is the closest thing to an obstacle observation time available.
            obstacle['timestamp'] = target.get('timestamp', FLOAT_FIELD_UNSET)
            # id is the obstacle's index in THIS cycle's emitted list, not the
            # source target's id. nepi_interfaces/Target declares an id field but
            # no producer assigns it -- node_if_ai_detector.py fills timestamp,
            # name, uid, confidence and the pixel fields and never id -- so it
            # arrives 0 on every target, and carrying it through would label every
            # obstacle 0. Index is the only definition that distinguishes the
            # obstacles in one message today. It is NOT stable across cycles; a
            # tracker that assigns a persistent id is the next developer's work.
            obstacle['id'] = len(obstacles)

            obstacles.append(obstacle)

    # use_navpose is declared and read above but inert: it asks for obstacle
    # POSITIONS in the NavPose frame, and Obstacle.msg carries no position field
    # to express one. The NavPose itself still reaches the consumer -- the node
    # puts it on the Obstacles message, not on each Obstacle.

    process_data_dict['obstacles'] = obstacles
    process_data_dict['obstacle_count'] = len(obstacles)
    process_data_dict['data_time'] = start_time
    process_data_dict['process_time'] = nepi_utils.get_time() - start_time

    return process_data_dict, process_controls_dict



PROCESSES_DICT['process_2'] = {'process_function': process_2,
                                'default_controls_dict': process_2_controls}



#########################
# Process Utility Functions
#########################

# create_processes_dict() and update_processes_dict() are deliberately absent.
#
# Both were part of the hand-rolled settings pattern this module has migrated off
# of, and ControlsIF replaces each of them outright:
#   create_processes_dict()  built {process_name: default_settings} for the node
#                            to hold and mutate. Each ControlsIF now builds and
#                            owns its own controls dict from the
#                            default_controls_dict below, and persists it to the
#                            param server itself, so there is nothing for the
#                            node to assemble.
#   update_processes_dict()  sanitized a settings dict coming back from the RUI,
#                            dropping unknown keys. ControlsIF's set_* callbacks
#                            do that per control, and nepi_controls clamps to
#                            bounds and rejects values outside options, so an
#                            after-the-fact sweep has nothing left to catch. Its
#                            inner loop also keyed off a nested
#                            'process_controls_dict' that no control set here
#                            ever contained.
# Nothing referenced either function -- this module had no importers before this
# migration.

def get_blank_data_dict():
    """Return a fresh per-cycle process data dict.

    Returns:
        dict: A deep copy of DATA_DICT, so a caller can mutate it without
            touching the module-level template.
    """
    return copy.deepcopy(DATA_DICT)


def get_blank_obstacle_dict():
    """Return a fresh per-obstacle dict with every field at its unset value.

    Every key is one field of msg/Obstacle.msg of the same name. Fill only the
    fields the process function can actually supply and leave the rest at the
    unset value they arrive with -- see docs/ADDING_OBSTACLE_MSG_FIELDS.md.

    Returns:
        dict: A deep copy of OBSTACLE_DICT.
    """
    return copy.deepcopy(OBSTACLE_DICT)
