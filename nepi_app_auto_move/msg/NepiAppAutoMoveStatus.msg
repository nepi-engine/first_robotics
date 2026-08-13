# This message details the AutoMove App Status Msg
#
# The app publishes this once per status cycle on <app>/status. Every command
# topic the app subscribes to hangs off that same <app> namespace, so a consumer
# that has this message has everything it needs to drive the app.

# Goto process states. The app reports exactly one of these in goto_state.
string GOTO_STATE_IDLE=idle
string GOTO_STATE_PLANNING=planning
string GOTO_STATE_MOVING=moving
string GOTO_STATE_COMPLETE=complete
string GOTO_STATE_CANCELLED=cancelled

#################
# Selected robot
#################
# Connect namespace of the app's ConnectRBXDeviceIF, FULLY QUALIFIED -- this is
# where the RUI's Nepi_IF_ConnectRBX mounts and where select_topic is published.
# Fully qualified because the RUI appends '/status' and hands the result to
# rosbridge, where a name with no leading slash resolves at the global root
# instead of under /<prefix>/<device_id>.
string rbx_connect_namespace
# Namespace of the RBX device currently selected on that connect, 'None' if none
string rbx_namespace
# True while that device is publishing DeviceRBXStatus
bool rbx_connected
# True when the selected device reports itself ready to accept a command
bool rbx_ready

#################
# Selected image source
#################
# Connect namespace of the app's ConnectImageIF, FULLY QUALIFIED, for the same
# reason as rbx_connect_namespace above
string image_connect_namespace
# Topic of the image currently selected on that connect, 'None' if none
string image_topic
# True while that image source is publishing ImageStatus
bool image_connected
# Size of the selected image as its ImageStatus reports it. The click pixel on
# this message is expressed in this raster.
int32 image_width_px
int32 image_height_px

#################
# Resolved companion topics
#################
# Each topic is derived from the selected image and reported with a found flag.
# A missing companion degrades the app rather than failing it: the flag reports
# false, the topic string is what the app looked for, and every feature that
# depends on it goes inert while the rest of the app keeps running.
#
# depth_map and depth_map_image are derived by NAMESPACE. Sibling data products
# share a parent namespace, so the depth map for <P>/color_image is <P>/depth_map
# and its colourized render is <P>/depth_map/depth_map_image. Note that
# DepthMapStatus.image_topic does NOT name the depth map image -- it names the
# sibling colour image -- so the image topic is never derived from that field.
string depth_map_topic
bool depth_map_found
string depth_map_image_topic
bool depth_map_image_found

# objects and targets are NOT namespace-derivable. An AI detector publishes them
# under its own node namespace and fans the same messages out on
# <base>/all/detections and <base>/all/targets. The app subscribes to the
# fan-out and matches on the message's source_topic; the topic reported here is
# the producing detector's own topic, learned from the matching message's
# process_namespace, and is empty until one arrives.
string objects_topic
bool objects_found

string targets_topic
bool targets_found

# obstacles match on the resolved DEPTH MAP topic, not on the image topic -- an
# obstacles process consumes depth maps, so that is what its source_topic names.
# Empty and false when no obstacles process is running for this depth map, or
# when the obstacles message package is not installed on this device.
string obstacles_topic
bool obstacles_found

#################
# Image viewer wiring
#################
# Topic the RUI image viewer publishes nepi_interfaces/ImageMouseEvent clicks on.
# The RUI passes this to Nepi_IF_ImageViewer as its mouse_event_topic prop.
string mouse_event_topic
# Overlay image the app's image pub node publishes, the topic the RUI viewer
# displays. Empty until an image source is selected and the node is publishing.
string overlay_image_topic

#################
# Overlay controls
#################
# How transparent the depth map image render is drawn over the source frame,
# 0.0 fully opaque through 1.0 invisible. The image pub node blends at
# (1.0 - transparency), so this reads the way the RUI slider labels it.
# Meaningful only while show_depth_map_enabled is true.
float32 depth_map_transparency

# One enable flag per overlay layer. A disabled layer is skipped in the render
# rather than drawn transparent, so it costs nothing.
bool show_depth_map_enabled
bool show_objects_enabled
bool show_targets_enabled
bool show_obstacles_enabled
bool show_crosshair_enabled

#################
# Click to goto
#################
# Last clicked pixel, in the selected image's raster (image_width_px by
# image_height_px above). The ratios are the same point expressed 0.0-1.0, which
# is what the image pub node draws the crosshair from -- the overlay frame is
# the source frame's size, which need not match the viewed stream's size.
nepi_interfaces/ImagePixel last_click_pixel
float32 last_click_x_ratio
float32 last_click_y_ratio
# False until the first click of this session
bool has_click
# True when the last click landed on a pixel with a valid depth return. False
# leaves the goto values below unchanged from the previous click.
bool click_depth_valid

# Requested relative move in the robot body frame: x forward, y left, z up, in
# METERS -- the nepi_interfaces/GotoPosition convention. Depth map arrays carry
# MILLIMETRES; the app converts explicitly where it reads them.
float32 goto_x_m
float32 goto_y_m
float32 goto_z_m
# Largest forward distance a single click may request, in meters. A click
# farther than this is scaled back along the same bearing.
float32 max_move_m
# True when the last click was scaled back to max_move_m
bool goto_clamped
# Human readable account of the last click, e.g. why no goto values were set
string click_msg

#################
# Goto process
#################
# One of the GOTO_STATE_* constants above
string goto_state
# Human readable account of the current or last goto process
string goto_msg
# Index of the move step being executed, and how many the plan holds
int32 goto_step
int32 goto_step_count

#################
# App controls
#################
# Per-control data is NOT carried here. It travels in the
# nepi_interfaces/ControlsStatus message the app's ControlsIF publishes at
# <controls_namespace>/status, exactly as in ObstaclesStatus.msg.
#
# The namespace is FULLY QUALIFIED, for the same rosbridge reason as
# rbx_connect_namespace above.
string controls_namespace
# True once that ControlsIF has completed initialization
bool controls_ready
