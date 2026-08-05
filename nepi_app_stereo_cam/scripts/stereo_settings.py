"""Stereo depth 'process' module -- control definitions and per-cycle process functions.

Same registry shape as nepi_sdk/nepi_stab_pt.py and the sibling
nepi_app_obstacles/scripts/nepi_obstacles.py:

  * PROCESSES_DICT        -- registry of named depth algorithms, each with a
                             process_function + default_controls_dict.
  * DEFAULT_PROCESS       -- the process the node selects before an operator picks.
  * get_blank_data_dict() -- fresh runtime data dict for one process call.

What changed from the pre-migration form of this module, and why it exists in this
shape: each process used to carry a hand-rolled default_settings_dict of raw
values that the node had to sanitize itself (create_processes_dict /
update_processes_dict), with a nested 'stereo_controls_dict' the RUI populated by
convention, and every value edited from the RUI arrived as a float regardless of
what it actually was. Here each default_controls_dict is a nepi_controls CONTROLS
INIT DICT instead -- the declarative form nepi_sdk/nepi_controls.create_controls_dict()
consumes -- and stereo_cam_app_node.py hands each set to its own ControlsIF. The IF
then owns type/bounds/options validation, RUI presentation, and param persistence,
which is why the hand-rolled create_processes_dict / update_processes_dict pair is
gone rather than ported. See the module note at the bottom for what each removed
function was replaced by.

Init dict key schema, per nepi_controls.create_controls_dict():
  'type'      REQUIRED, one of nepi_controls.CONTROL_TYPES. An entry whose type
              is missing or unrecognized is silently dropped.
  'default'   REQUIRED by every type except Trigger. Menu takes an int index,
              Selection a string that must appear in options.
  'options'   REQUIRED by Menu / Selection / Selections.
  'bounds'    [min, max] for Int / Float / FloatSlider. -999 means no limit.
  Any remaining key is copied through only if it is also a nepi_interfaces/Control
  field: 'display_name', 'description', 'hidden', 'round_value', 'round_display'.
  ('value_round' / 'display_round', as spelled in nepi_controls.EXAMPLE_INIT_DICT,
  are NOT Control fields and are ignored.)

WHY THE CONSTRAINED VALUES ARE Selection AND NOT Int
Three of the block-matching values are not free integers, they are members of a
fixed legal set, and stereo_library rejects or silently ignores anything else:
  num_disparities     _build_matcher raises ValueError unless it is a positive
                      multiple of 16, which would take the depth loop's whole
                      pass down (caught and throttle-warned by depthCb, so the
                      operator sees no depth and one warning per 5 s).
  block_size          _build_matcher raises ValueError unless it is odd. StereoBM
                      additionally requires >= 5, so bm_1 offers no smaller value.
  median_filter_size  compute_depth_map applies the filter only when the value is
                      odd and >= 3. An even value is not an error, it just quietly
                      turns the filter off, which is worse than an error.
A free Int box lets an operator type an illegal value and find out from a log
line. A Selection cannot express one: nepi_controls.set_control_value rejects any
string not in string_options. The values come back as STRINGS and are int()ed in
_build_cfg below.

Three nepi_controls types are deliberately unused:
  FloatSliders -- create_controls_dict()'s FloatSliders branch references an
                  undefined name `value` (nepi_controls.py, `if value[0] >
                  value[1]`), the surrounding except swallows the NameError, and
                  the control is dropped. min_depth_mm / max_depth_mm are the
                  natural range window, so they are written as two Float controls
                  until that branch is fixed.
  Trigger      -- Nepi_IF_Controls.js publishes UpdateString to
                  set_trigger_control_value, which ControlsIF subscribes to as
                  UpdateTrigger, so a Trigger control cannot be fired from the
                  RUI today. Nothing here depends on one.
  runtime hiding -- nepi_controls.set_control_hidden() does `hidden = str(hidden)`,
                  writing 'True'/'False' into a field nepi_interfaces/Control
                  declares bool, and Nepi_IF_Controls ignores ControlsStatus.hidden
                  anyway. A control's 'hidden' works only as authored below.

Each process_function(left_image, right_image, stereo_data_dict,
process_controls_dict) is called once per depth cycle by stereo_cam_app_node.py.
process_controls_dict is a flat {control_name: current_value} snapshot the node
reads out of that process's ControlsIF, so a control is read here by exactly the
key it is authored under below -- a missing key means the control was dropped at
create_controls_dict() time, not that the operator changed something. The
positional signature and the DATA_DICT contract are unchanged from the
pre-migration form; only the fourth argument's contents (and its name) moved from
a nested settings dict to a flat controls snapshot.

Each process computes a depth map (mm, float32) via
stereo_library.compute_depth_map and stashes it in stereo_data_dict['depth_map'];
the node then hands that to DepthMapIF, which publishes the raw map plus a
colorized image of it.
"""

import copy

from nepi_sdk import nepi_utils

from stereo_library import compute_depth_map


# Runtime data template (mirrors nepi_stab_pt.DATA_DICT).
DATA_DICT = {
    # Required Fields
    'data_time': 0.0,
    'process_time': 0.0,

    # Depth results
    'depth_map': None,        # (H,W) float32 mm; 0.0 == invalid
    'valid_ratio': 0.0,       # fraction of pixels with valid depth
    'result_min_depth_mm': 0.0,
    'result_max_depth_mm': 0.0,
    # Add Custom Fields Here
}


PROCESSES_DICT = dict()

DEFAULT_PROCESS = 'sgbm_1'


# Legal option sets for the three constrained values, shared by both processes
# where the constraint is the same. Authored as strings because that is what a
# Selection control's value IS -- nepi_controls stores set_string and hands the
# string straight back.
#
# NUM_DISPARITIES_OPTIONS spans 16..256: 16 is the smallest legal step and 256 is
# already a wide search for a 60 mm baseline. BLOCK_SIZE_OPTIONS_SGBM covers the
# 3-15 SGBM window range plus 21 for very low-texture scenes;
# BLOCK_SIZE_OPTIONS_BM starts at 5 because cv2.StereoBM_create rejects a smaller
# window. MEDIAN_FILTER_OPTIONS leads with '0' because 0 is how the filter is
# turned off, and every other entry is odd and >= 3, the only sizes
# compute_depth_map actually applies.
NUM_DISPARITIES_OPTIONS = ['16', '32', '48', '64', '80', '96', '112', '128',
                           '160', '192', '224', '256']
BLOCK_SIZE_OPTIONS_SGBM = ['3', '5', '7', '9', '11', '13', '15', '21']
BLOCK_SIZE_OPTIONS_BM = ['5', '7', '9', '11', '13', '15', '17', '19', '21']
MEDIAN_FILTER_OPTIONS = ['0', '3', '5', '7', '9', '11']


# Stereo Process Functions

def _build_cfg(matcher, process_controls_dict):
    """Flatten a process controls snapshot into the flat dict compute_depth_map wants."""
    controls = process_controls_dict
    return {
        'matcher': matcher,
        # Authored as a control only by sgbm_1, where the channel count actually
        # changes the SGBM smoothness penalties. BM ignores it -- compute_depth_map
        # forces grayscale whenever the matcher is 'bm' -- so bm_1 offers no such
        # control and falls through to this default.
        'convert_to_grayscale': bool(controls.get('convert_to_grayscale', True)),
        # Camera geometry. Operator-visible, but normally written by the node from
        # the loaded calibration -- see stereo_cam_app_node.applyCalibGeometry().
        'baseline_mm': float(controls.get('baseline_mm', 60.0)),
        'focal_length_px': float(controls.get('focal_length_px', 700.0)),
        # Block matching. The three Selection controls arrive as strings.
        'min_disparity': int(controls.get('min_disparity', 0)),
        'num_disparities': int(controls.get('num_disparities', 128)),
        'block_size': int(controls.get('block_size', 5)),
        'uniqueness_ratio': int(controls.get('uniqueness_ratio', 10)),
        'speckle_window_size': int(controls.get('speckle_window_size', 100)),
        'speckle_range': int(controls.get('speckle_range', 2)),
        'disp12_max_diff': int(controls.get('disp12_max_diff', 1)),
        'pre_filter_cap': int(controls.get('pre_filter_cap', 63)),
        # Depth post-processing
        'min_depth_mm': float(controls.get('min_depth_mm', 50.0)),
        'max_depth_mm': float(controls.get('max_depth_mm', 20000.0)),
        'median_filter_size': int(controls.get('median_filter_size', 5)),
    }


def _run(matcher, left_image, right_image, stereo_data_dict, process_controls_dict):
    """Shared body for the block-matching processes."""
    start_time = nepi_utils.get_time()
    cfg = _build_cfg(matcher, process_controls_dict)

    depth = compute_depth_map(left_image, right_image, cfg)  # (H,W) float32 mm

    valid = depth[depth > 0]
    stereo_data_dict['data_time'] = start_time
    stereo_data_dict['process_time'] = nepi_utils.get_time() - start_time
    stereo_data_dict['depth_map'] = depth
    stereo_data_dict['valid_ratio'] = float(valid.size) / depth.size if depth.size else 0.0
    stereo_data_dict['result_min_depth_mm'] = float(valid.min()) if valid.size else 0.0
    stereo_data_dict['result_max_depth_mm'] = float(valid.max()) if valid.size else 0.0
    return stereo_data_dict, process_controls_dict



############################

# Process 1: SGBM (semi-global, higher quality, 3 layer)
#
# Insertion order sets the RUI display order: camera geometry first (what the
# calibration writes), then the block matcher, then depth post-processing.
sgbm_1_controls = {

    'baseline_mm': {
        'type': 'Float', 'default': 60.0, 'bounds': [1.0, 5000.0], 'round_value': 2,
        'display_name': 'Baseline (mm)',
        'description': 'Distance between the two camera optical centers. Written by the calibration solve; edit only to run without a calibration.',
        'hidden': False},

    'focal_length_px': {
        'type': 'Float', 'default': 700.0, 'bounds': [1.0, 100000.0], 'round_value': 2,
        'display_name': 'Focal Length (px)',
        'description': 'Rectified focal length in pixels. Written by the calibration solve; edit only to run without a calibration.',
        'hidden': False},

    'convert_to_grayscale': {
        'type': 'Bool', 'default': True,
        'display_name': 'Match In Grayscale',
        'description': 'Convert the pair to grayscale before matching. Off matches all three color channels: slower, and only worth it on low-texture color scenes.',
        'hidden': False},

    'min_disparity': {
        'type': 'Int', 'default': 0, 'bounds': [0, 256],
        'display_name': 'Min Disparity',
        'description': 'Smallest disparity searched. Raise only when nothing in the scene is close enough to produce a near-zero disparity.',
        'hidden': False},

    'num_disparities': {
        'type': 'Selection', 'default': '128', 'options': NUM_DISPARITIES_OPTIONS,
        'display_name': 'Num Disparities',
        'description': 'Width of the disparity search, in pixels. Must be a multiple of 16, so the legal values are offered as a list. Larger measures closer objects and costs more.',
        'hidden': False},

    'block_size': {
        'type': 'Selection', 'default': '5', 'options': BLOCK_SIZE_OPTIONS_SGBM,
        'display_name': 'Block Size',
        'description': 'Side length of the matching window. Must be odd, so the legal values are offered as a list. Larger is smoother and blurs depth edges.',
        'hidden': False},

    'uniqueness_ratio': {
        'type': 'Int', 'default': 10, 'bounds': [0, 100],
        'display_name': 'Uniqueness Ratio (%)',
        'description': 'A match must beat the second-best candidate by this percent margin or it is rejected. 0 disables the check.',
        'hidden': False},

    'speckle_window_size': {
        'type': 'Int', 'default': 100, 'bounds': [0, 500],
        'display_name': 'Speckle Window Size (px)',
        'description': 'Largest connected disparity blob treated as noise and cleared. 0 disables speckle filtering.',
        'hidden': False},

    'speckle_range': {
        'type': 'Int', 'default': 2, 'bounds': [0, 64],
        'display_name': 'Speckle Range',
        'description': 'Disparity variation allowed within one blob while speckle filtering.',
        'hidden': False},

    'disp12_max_diff': {
        'type': 'Int', 'default': 1, 'bounds': [-1, 128],
        'display_name': 'Disp12 Max Diff (px)',
        'description': 'Left-right consistency tolerance, in disparity pixels. -1 disables the occlusion check.',
        'hidden': False},

    'pre_filter_cap': {
        'type': 'Int', 'default': 63, 'bounds': [1, 63],
        'display_name': 'Pre-Filter Cap',
        'description': 'Clamp applied to image gradients before matching. Higher keeps sharper texture detail; 1-63 is the legal range.',
        'hidden': False},

    'min_depth_mm': {
        'type': 'Float', 'default': 50.0, 'bounds': [1.0, 100000.0], 'round_value': 1,
        'display_name': 'Min Depth (mm)',
        'description': 'Depths closer than this are marked invalid. Also the near end of the colorized depth image range.',
        'hidden': False},

    'max_depth_mm': {
        'type': 'Float', 'default': 20000.0, 'bounds': [1.0, 100000.0], 'round_value': 1,
        'display_name': 'Max Depth (mm)',
        'description': 'Depths farther than this are marked invalid. Also the far end of the colorized depth image range.',
        'hidden': False},

    'median_filter_size': {
        'type': 'Selection', 'default': '5', 'options': MEDIAN_FILTER_OPTIONS,
        'display_name': 'Median Filter Size (px)',
        'description': 'Median filter applied to the depth map to drop isolated pixels without blurring edges. 0 disables it; every other value is odd because an even one would silently disable it.',
        'hidden': False},

    }


def sgbm_1(left_image, right_image, stereo_data_dict, process_controls_dict):
    return _run('sgbm', left_image, right_image, stereo_data_dict, process_controls_dict)


PROCESSES_DICT['sgbm_1'] = {'process_function': sgbm_1,
                            'default_controls_dict': sgbm_1_controls}


############################

# Process 2: BM (block matching, faster, noisier, grayscale)
#
# Same control set as sgbm_1 with two deliberate differences: no
# convert_to_grayscale (BM is grayscale-only, and compute_depth_map converts for
# it regardless, so a toggle here would drive nothing), and a larger block_size
# default off a list that starts at 5 because cv2.StereoBM_create rejects a
# smaller window.
bm_1_controls = {

    'baseline_mm': {
        'type': 'Float', 'default': 60.0, 'bounds': [1.0, 5000.0], 'round_value': 2,
        'display_name': 'Baseline (mm)',
        'description': 'Distance between the two camera optical centers. Written by the calibration solve; edit only to run without a calibration.',
        'hidden': False},

    'focal_length_px': {
        'type': 'Float', 'default': 700.0, 'bounds': [1.0, 100000.0], 'round_value': 2,
        'display_name': 'Focal Length (px)',
        'description': 'Rectified focal length in pixels. Written by the calibration solve; edit only to run without a calibration.',
        'hidden': False},

    'min_disparity': {
        'type': 'Int', 'default': 0, 'bounds': [0, 256],
        'display_name': 'Min Disparity',
        'description': 'Smallest disparity searched. Raise only when nothing in the scene is close enough to produce a near-zero disparity.',
        'hidden': False},

    'num_disparities': {
        'type': 'Selection', 'default': '128', 'options': NUM_DISPARITIES_OPTIONS,
        'display_name': 'Num Disparities',
        'description': 'Width of the disparity search, in pixels. Must be a multiple of 16, so the legal values are offered as a list. Larger measures closer objects and costs more.',
        'hidden': False},

    'block_size': {
        'type': 'Selection', 'default': '15', 'options': BLOCK_SIZE_OPTIONS_BM,
        'display_name': 'Block Size',
        'description': 'Side length of the matching window. Must be odd and at least 5 for BM, so the legal values are offered as a list. BM wants a larger window than SGBM.',
        'hidden': False},

    'uniqueness_ratio': {
        'type': 'Int', 'default': 10, 'bounds': [0, 100],
        'display_name': 'Uniqueness Ratio (%)',
        'description': 'A match must beat the second-best candidate by this percent margin or it is rejected. 0 disables the check.',
        'hidden': False},

    'speckle_window_size': {
        'type': 'Int', 'default': 100, 'bounds': [0, 500],
        'display_name': 'Speckle Window Size (px)',
        'description': 'Largest connected disparity blob treated as noise and cleared. 0 disables speckle filtering.',
        'hidden': False},

    'speckle_range': {
        'type': 'Int', 'default': 2, 'bounds': [0, 64],
        'display_name': 'Speckle Range',
        'description': 'Disparity variation allowed within one blob while speckle filtering.',
        'hidden': False},

    'disp12_max_diff': {
        'type': 'Int', 'default': 1, 'bounds': [-1, 128],
        'display_name': 'Disp12 Max Diff (px)',
        'description': 'Left-right consistency tolerance, in disparity pixels. -1 disables the occlusion check.',
        'hidden': False},

    'pre_filter_cap': {
        'type': 'Int', 'default': 63, 'bounds': [1, 63],
        'display_name': 'Pre-Filter Cap',
        'description': 'Clamp applied to image gradients before matching. Higher keeps sharper texture detail; 1-63 is the legal range.',
        'hidden': False},

    'min_depth_mm': {
        'type': 'Float', 'default': 50.0, 'bounds': [1.0, 100000.0], 'round_value': 1,
        'display_name': 'Min Depth (mm)',
        'description': 'Depths closer than this are marked invalid. Also the near end of the colorized depth image range.',
        'hidden': False},

    'max_depth_mm': {
        'type': 'Float', 'default': 20000.0, 'bounds': [1.0, 100000.0], 'round_value': 1,
        'display_name': 'Max Depth (mm)',
        'description': 'Depths farther than this are marked invalid. Also the far end of the colorized depth image range.',
        'hidden': False},

    'median_filter_size': {
        'type': 'Selection', 'default': '5', 'options': MEDIAN_FILTER_OPTIONS,
        'display_name': 'Median Filter Size (px)',
        'description': 'Median filter applied to the depth map to drop isolated pixels without blurring edges. 0 disables it; every other value is odd because an even one would silently disable it.',
        'hidden': False},

    }


def bm_1(left_image, right_image, stereo_data_dict, process_controls_dict):
    return _run('bm', left_image, right_image, stereo_data_dict, process_controls_dict)


PROCESSES_DICT['bm_1'] = {'process_function': bm_1,
                          'default_controls_dict': bm_1_controls}


#########################
# Stereo Utility Functions
#########################

# create_processes_dict() and update_processes_dict() are deliberately absent.
#
# Both were part of the hand-rolled settings pattern this module has migrated off
# of, and ControlsIF replaces each of them outright:
#   create_processes_dict()  built {process_name: default_settings} for the node
#                            to hold as a 'processes_dict' param and mutate. Each
#                            ControlsIF now builds and owns its own controls dict
#                            from the default_controls_dict above, and persists it
#                            to the param server itself, so there is nothing for
#                            the node to assemble and no 'processes_dict' param.
#   update_processes_dict()  sanitized a settings dict coming back from the RUI,
#                            dropping unknown keys and casting values to the type
#                            of the existing default. ControlsIF's set_* callbacks
#                            do that per control on the way in: nepi_controls
#                            rejects a value outside bounds, rejects a string not
#                            in options, and there is one typed topic per control
#                            type instead of one UpdateFloat for everything -- so
#                            an after-the-fact sweep has nothing left to catch.
#                            The nested 'stereo_controls_dict' it copied through
#                            is gone with it; min_depth_mm, max_depth_mm and
#                            median_filter_size are now ordinary controls
#                            alongside the rest.
#
# The two callers both moved to the ControlsIF path: stereo_cam_app_node.py's
# initCb / reloadProcessesCb / setProcessControlValueCb held every call site, and
# setProcessControlValueCb is gone entirely.

def get_blank_data_dict():
    return copy.deepcopy(DATA_DICT)
