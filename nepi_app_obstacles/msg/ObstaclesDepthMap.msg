# This message provides the per-cycle depth map segmentation from a single
# source. It deliberately carries no obstacle list -- the Obstacles message
# published on the sibling topic carries that, so consumers that only need the
# obstacle list never pay for these images.
#
# The two messages are published back to back from the same process cycle.
# Pair them on source_topic plus source_timestamp when the pairing matters.

float64 timestamp

string process_name
string process_namespace

string source_topic
float64 source_timestamp

string navpose_frame
nepi_interfaces/NavPose navpose_msg

# Per-cycle depth map segmentation, so the image publisher node overlays
# exactly the maps the obstacles were derived from. Both are 32FC1 range
# images the same size as the source depth map, with non-member pixels set to
# NaN. Empty (0x0) images when the segmentation was not produced this cycle.
sensor_msgs/Image depth_map_ground
sensor_msgs/Image depth_map_obstacles
