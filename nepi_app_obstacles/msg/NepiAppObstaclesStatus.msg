# This message details the Obstacles App Status Msg

# App enabled state
bool enabled
# Available options (published so the RUI menu always matches the node)
string[] options
# Currently selected option
string selected_option
# Current value setting
float32 value

#################
# Operator Selections
#################
# The only three topics an operator picks on this app's RUI page. Each is the
# selected_topic of one Nepi_IF_Connect* selector, or "None" when nothing is
# connected.

# Selected depth map source topic, or "None" when nothing is connected
string depth_map_topic
# Selected targets source topic, or "None" when nothing is connected
string targets_topic
# Selected NavPose source topic, or "None" when nothing is connected
string navpose_topic

#################
# Derived Image Topics
#################
# Every image this app reports is DERIVED from one of the selections above and
# never selected directly -- there is no image selector on the page. Each is
# read off the selected source's own live status message, is rejected unless
# that status belongs to the currently selected source, and is "None" whenever
# the derivation cannot name a real topic. The RUI mounts a viewer only on a
# topic that is not "None", so "None" means the viewer stays unmounted rather
# than mounting a dead stream.

# Depth map image topic belonging to the depth map selected above. Derived from
# that depth map's DepthMapStatus: image_topic when it resolves under the
# selected depth map namespace, otherwise the depth map image DepthMapIF
# publishes at <depth_map>/depth_map_image when img_pub_enabled is set. "None"
# when nothing is connected or the selected depth map reports no associated
# image. Never the color image -- see obstacles_app_node.py
# getDepthMapImageTopic()
string depth_map_image_topic
# Color image topic belonging to the depth map selected above. Derived from the
# same DepthMapStatus, taking exactly the image_topic that getDepthMapImageTopic()
# rejects: the SIBLING color image, which every DepthMapIF producer in nepi_api
# reports and which does not resolve under the depth map namespace. "None" when
# nothing is connected, the selected depth map reports no sibling image, or that
# image resolves under the depth map namespace and so is a depth map image
# rather than a color image -- see obstacles_app_node.py getColorImageTopic()
string image_topic
# Targets image topic belonging to the targets source selected above. Derived
# from that source's TargetingStatus: the targets overlay image the detector
# image pub node publishes as a sibling of the detections image named in
# ProcessStatus.imaging_pub_topics. "None" when nothing is connected, the
# detector's imaging path is not running, it reports no pub topics, or the
# derived topic is not among the connected image topics -- see
# obstacles_app_node.py getTargetsImageTopic()
string targets_image_topic

#################
# Controls
#################
# Per-control data is NOT carried here. It travels in the
# nepi_interfaces/ControlsStatus message each ControlsIF publishes at
# <each controls namespace>/status, exactly as in ControlsSandboxStatus.msg. What this
# app adds over the sandbox is multiplicity: it owns one controls namespace per
# process, because nepi_obstacles.PROCESSES_DICT holds one control set per process
# and only one process is active at a time.
#
# Every namespace below is FULLY QUALIFIED. ControlsIF.get_namespace() returns a
# relative namespace ('app_obstacles/process_1', no leading slash), which resolves
# where the IF itself uses it but not when a RUI component appends '/status' and
# hands it to rosbridge -- see obstacles_app_node.py getControlsNamespace().

# Controls namespace of the process named in selected_process below, or "None"
# when there is no valid selection. The RUI mounts its single Nepi_IF_Controls
# here and nowhere else, which is what shows the operator exactly one process's
# controls: the inactive process's component is not mounted at all. Suppression
# cannot be done with the per-control hidden flag -- see
# obstacles_app_node.py applyActiveProcessVisibility()
string active_controls_namespace
# Every controls namespace this app owns, one per process in
# nepi_obstacles.PROCESSES_DICT order. Published for discoverability; the RUI
# binds to the field above, not to this list.
string[] controls_namespaces
# True only when every per-process ControlsIF has completed initialization
bool controls_ready

# Namespace of the example control set, a copy of nepi_app_controls_sandbox's
# demonstration controls that belongs to no process and drives nothing in this app.
# NepiAppObstacles.js mounts its Example Controls box here, in ADDITION to the
# single Nepi_IF_Controls it mounts on active_controls_namespace above -- this one
# is always mounted, since there is no active/inactive question about it.
string example_controls_namespace
# True once that ControlsIF has completed initialization. Reported separately from
# controls_ready above, which covers only the per-process sets.
bool example_controls_ready

#################
# Process Selector
#################
# The selector is this app's own state on this app's own topics, not a control:
# set_selected_process (std_msgs/String, the process NAME) and reload_processes
# (std_msgs/Empty), the same pair nepi_app_stereo_cam uses. NepiAppObstacles.js
# renders the dropdown and the Reload Processes button from the three fields below.

# Name of the active obstacle process, whose controls namespace is
# active_controls_namespace above
string selected_process
# Every process nepi_obstacles.PROCESSES_DICT registers, published so the RUI
# menu cannot drift from what the module actually offers
string[] available_processes
# False only while reload_processes is re-importing the module. The RUI disables
# the process dropdown while it is False.
bool process_ready
