# This message details the CustomStereo App Status Msg
# Fields mirror NepiCustomStereoApp.publish_status()

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
# Depth output
# Depth map topic published by this app's IDX interface (feeds the large viewer)
string depth_map_topic
# Max depth framerate (Hz)
float32 max_framerate
# Depth summary from the last processed frame
float32 valid_ratio
float32 result_min_depth_mm
float32 result_max_depth_mm
