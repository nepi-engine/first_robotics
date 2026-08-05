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

    # Outputs. obstacles is left as a plain list of dicts rather than
    # nepi_app_obstacles/Obstacle messages: msg/Obstacle.msg and msg/Obstacles.msg
    # exist in this package but are not listed in CMakeLists.txt
    # add_message_files(), so neither type is generated and neither can be
    # imported yet.
    'obstacles': [],
    'obstacle_count': 0,
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

    # Detection itself is not implemented. The depth map arrives as whatever dict
    # ConnectDepthMapIF hands its dataCB, and extracting an obstacle from it is
    # the app's remaining work -- writing a stand-in here would be inventing both
    # that dict's key names and a detection result. What IS wired and verifiable
    # is everything around it: the controls above are live and operator-settable,
    # and the inputs below are the current cycle's data.
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

    # Detection and fusion are not implemented -- same reason as process_1: the
    # depth map and targets dicts are the connect IFs' own contracts, and this
    # app has not defined how a target and a depth return combine into an
    # obstacle yet. The controls and the inputs are wired and verifiable.
    obstacles = []
    if depth_map_dict is not None and targets_dict is not None:
        pass

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
    return copy.deepcopy(DATA_DICT)
