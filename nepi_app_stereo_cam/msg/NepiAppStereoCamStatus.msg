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
# Selected process's tunable settings, flattened into parallel arrays so the RUI
# can render one editable box per variable (mirrors PanTiltAutoAppStatus
# auto_control_names / auto_control_values). Names and values are index-aligned;
# edits are sent back per-name via set_process_control_value (UpdateFloat).
string[] process_control_names
float32[] process_control_values

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
# Per-control data is NOT carried here. It travels in the
# nepi_interfaces/ControlsStatus message the app's example ControlsIF publishes at
# <example_controls_namespace>/status, exactly as in ControlsSandboxStatus.msg.
# Nothing to do with the process_control_names / process_control_values arrays
# above, which are this app's own flattened process settings.
#
# The namespace is FULLY QUALIFIED. ControlsIF.get_namespace() returns a relative
# namespace ('app_stereo_cam/example_controls', no leading slash), which resolves
# where the IF itself uses it but not when a RUI component appends '/status' and
# hands it to rosbridge -- see stereo_cam_app_node.py getExampleControlsNamespace().

# Namespace of the example control set copied from nepi_app_controls_sandbox,
# where NepiAppStereoCam.js mounts its Example Controls box
string example_controls_namespace
# True once that ControlsIF has completed initialization
bool example_controls_ready