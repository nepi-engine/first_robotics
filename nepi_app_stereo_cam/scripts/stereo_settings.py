"""Stereo depth 'process' module -- settings back-and-forth with the RUI.

Patterned on nepi_stab_pt.py:
  * PROCESSES_DICT           -- registry of named depth algorithms, each with a
                                process_function + default_settings_dict.
  * create_processes_dict()  -- fresh {process_name: default_settings} for the node.
  * update_processes_dict()  -- sanitize settings coming back from the RUI (only
                                known keys survive; the nested *_controls_dict is
                                auto-populated in the RUI).
  * get_blank_data_dict()    -- fresh runtime data dict for a process call.

Each process_function(left, right, data_dict, settings_dict) computes a depth
map (mm, float32) via stereo_library.compute_depth_map and stashes it in
data_dict['depth_map']; the node then hands that to DepthMapIF, which publishes
the raw map plus a colorized image of it.
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


# Stereo Process Functions

def _build_cfg(matcher, settings_dict):
    """Flatten a process settings_dict into the flat dict compute_depth_map wants."""
    controls = settings_dict['stereo_controls_dict']
    return {
        'matcher': matcher,
        'convert_to_grayscale': True,
        # Camera geometry
        'baseline_mm': settings_dict['baseline_mm'],
        'focal_length_px': settings_dict['focal_length_px'],
        # Block matching
        'min_disparity': settings_dict['min_disparity'],
        'num_disparities': settings_dict['num_disparities'],
        'block_size': settings_dict['block_size'],
        'uniqueness_ratio': settings_dict['uniqueness_ratio'],
        'speckle_window_size': settings_dict['speckle_window_size'],
        'speckle_range': settings_dict['speckle_range'],
        'disp12_max_diff': settings_dict['disp12_max_diff'],
        'pre_filter_cap': settings_dict['pre_filter_cap'],
        # Depth post-processing (RUI-tunable custom fields)
        'min_depth_mm': controls['min_depth_mm'],
        'max_depth_mm': controls['max_depth_mm'],
        'median_filter_size': controls['median_filter_size'],
    }


def _run(matcher, left_image, right_image, stereo_data_dict, stereo_settings_dict):
    """Shared body for the block-matching processes."""
    start_time = nepi_utils.get_time()
    cfg = _build_cfg(matcher, stereo_settings_dict)

    depth = compute_depth_map(left_image, right_image, cfg)  # (H,W) float32 mm

    valid = depth[depth > 0]
    stereo_data_dict['data_time'] = start_time
    stereo_data_dict['process_time'] = nepi_utils.get_time() - start_time
    stereo_data_dict['depth_map'] = depth
    stereo_data_dict['valid_ratio'] = float(valid.size) / depth.size if depth.size else 0.0
    stereo_data_dict['result_min_depth_mm'] = float(valid.min()) if valid.size else 0.0
    stereo_data_dict['result_max_depth_mm'] = float(valid.max()) if valid.size else 0.0
    return stereo_data_dict, stereo_settings_dict



# Process 1: SGBM (semi-global, higher quality, 3 layer)

sgbm_1_settings = {
    # Required Fields (camera geometry, from calibration)
    'baseline_mm': 60.0,
    'focal_length_px': 700.0,
    'convert_to_grayscale': True,

    # Block matching
    'min_disparity': 0,
    'num_disparities': 128,       # multiple of 16
    'block_size': 5,              # odd
    'uniqueness_ratio': 10,
    'speckle_window_size': 100,
    'speckle_range': 2,
    'disp12_max_diff': 1,
    'pre_filter_cap': 63,

    # Custom Fields. Automatically Populated in RUI
    'stereo_controls_dict': {
        'min_depth_mm': 50.0,
        'max_depth_mm': 20000.0,
        'median_filter_size': 5,
    }
}


def sgbm_1(left_image, right_image, stereo_data_dict, stereo_settings_dict):
    return _run('sgbm', left_image, right_image, stereo_data_dict, stereo_settings_dict)


PROCESSES_DICT['sgbm_1'] = {
    'process_function': sgbm_1,
    'default_settings_dict': sgbm_1_settings}


# Process 2: BM (block matching, faster, noisier, grayscale)

bm_1_settings = {
    # Required Fields (camera geometry, from calibration)
    'baseline_mm': 60.0,
    'focal_length_px': 700.0,

    # Block matching
    'min_disparity': 0,
    'num_disparities': 128,       # multiple of 16
    'block_size': 15,             # odd; BM likes larger windows
    'uniqueness_ratio': 10,
    'speckle_window_size': 100,
    'speckle_range': 2,
    'disp12_max_diff': 1,
    'pre_filter_cap': 63,

    # Custom Fields. Automatically Populated in RUI
    'stereo_controls_dict': {
        'min_depth_mm': 50.0,
        'max_depth_mm': 20000.0,
        'median_filter_size': 5,
    }
}


def bm_1(left_image, right_image, stereo_data_dict, stereo_settings_dict):
    return _run('bm', left_image, right_image, stereo_data_dict, stereo_settings_dict)


PROCESSES_DICT['bm_1'] = {
    'process_function': bm_1,
    'default_settings_dict': bm_1_settings
    }


# Stereo Utility Functions

def create_processes_dict():
    processes_dict = dict()
    for process_name in PROCESSES_DICT.keys():
        processes_dict[process_name] = PROCESSES_DICT[process_name]['default_settings_dict']
    return processes_dict


def update_processes_dict(stereo_processes_dict):
    clean_dict = create_processes_dict()
    for process in clean_dict.keys():
        if process in stereo_processes_dict.keys():
            # Copy recognized top-level keys (skip the nested controls dict).
            for key in clean_dict[process].keys():
                if key in stereo_processes_dict[process].keys() and key != 'stereo_controls_dict':
                    clean_dict[process][key] = stereo_processes_dict[process][key]
            # Copy recognized keys inside the RUI-populated controls dict.
            for key in clean_dict[process]['stereo_controls_dict'].keys():
                if key in stereo_processes_dict[process].get('stereo_controls_dict', {}).keys():
                    clean_dict[process]['stereo_controls_dict'][key] = \
                        stereo_processes_dict[process]['stereo_controls_dict'][key]
    return clean_dict


def get_blank_data_dict():
    return copy.deepcopy(DATA_DICT)
