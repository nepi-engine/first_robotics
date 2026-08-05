# This message details the Stereo Cam App Status Msg
# Fields mirror NepiStereoCamApp.publish_status()

#####################
# Stereo processes dropdown (nepi processes pattern)
# Available process names (published so the RUI menu always matches the node)
string[] available_processes
# Currently selected stereo process
string selected_process
# False while a reload_processes is in progress
bool process_ready

#####################
# Process Controls
#####################
# Per-control data is NOT carried here. It travels in the
# nepi_interfaces/ControlsStatus message each ControlsIF publishes at
# <each controls namespace>/status, exactly as in ControlsSandboxStatus.msg. What
# this app adds over the sandbox is multiplicity: it owns one controls namespace
# per process, because stereo_settings.PROCESSES_DICT holds one control set per
# process and only one process is active at a time.
#
# Every namespace below is FULLY QUALIFIED. ControlsIF.get_namespace() returns a
# relative namespace ('app_stereo_cam/sgbm_1', no leading slash), which resolves
# where the IF itself uses it but not when a RUI component appends '/status' and
# hands it to rosbridge -- see stereo_cam_app_node.py getControlsNamespace().

# Controls namespace of the process named in selected_process above, or "None"
# when there is no valid selection. The RUI mounts its single Nepi_IF_Controls
# here and nowhere else, which is what shows the operator exactly one process's
# controls: the inactive process's component is not mounted at all. Suppression
# cannot be done with the per-control hidden flag -- see
# stereo_cam_app_node.py setupProcessControlsIFs()
string active_controls_namespace
# Every process controls namespace this app owns, one per process in
# stereo_settings.PROCESSES_DICT order. Published for discoverability; the RUI
# binds to the field above, not to this list.
string[] controls_namespaces
# True only when every per-process ControlsIF has completed initialization
bool controls_ready

#####################
# Left / Right IDX camera selectors
# True when the app's ConnectIDXDeviceIF is connected to the selected device
bool left_cam_connected
bool right_cam_connected
# Selected device namespaces ('None' if unselected)
string left_cam_topic
string right_cam_topic
# Color image topics feeding the two small image viewers ('None' if unselected)
string left_image_topic
string right_image_topic

#####################
# Stereo calibration (RUI-driven; see calibrate.StereoCalibrator)
# True once a calibration .npz is loaded and frames are being rectified
bool calib_loaded
# Path the .npz is written to / read from
string calib_file
# Chessboard description (INNER corner counts + printed square size)
int32 calib_board_cols
int32 calib_board_rows
float32 calib_square_mm
# Board views captured so far in this calibration session
int32 calib_capture_count
# Minimum captures needed before a solve is allowed
int32 calib_min_captures
# Last calibration action result, shown verbatim in the RUI
string calib_message
# Geometry from the loaded calibration (0.0 when none loaded)
float32 calib_focal_length_px
float32 calib_baseline_mm
# Rectified epipolar RMS from the last solve; < 1 px is good
float32 calib_epipolar_rms_px

#####################
# Depth output (published by this app's DepthMapIF)
# Raw depth map topic: sensor_msgs/Image, 32FC1, millimeters, nan == invalid
string depth_map_topic
# Colorized depth image topic (feeds the large RUI viewer)
string depth_map_image_topic
# Max depth framerate (Hz); settable via set_max_framerate
float32 max_framerate
# Depth summary from the last processed frame
float32 valid_ratio
float32 result_min_depth_mm
float32 result_max_depth_mm

#####################
# Example Controls
#####################
# Namespace of the example control set, a copy of nepi_app_controls_sandbox's
# demonstration controls that belongs to no process and drives nothing in this app.
# NepiAppStereoCam.js mounts its Example Controls box here, in ADDITION to the
# single Nepi_IF_Controls NepiAppStereoCam-Controls.js mounts on
# active_controls_namespace above -- this one is always mounted, since there is no
# active/inactive question about it. Same fully-qualified form, and the same reason
# for it -- see stereo_cam_app_node.py getControlsNamespace().
string example_controls_namespace
# True once that ControlsIF has completed initialization. Reported separately from
# controls_ready above, which covers only the per-process sets.
bool example_controls_ready