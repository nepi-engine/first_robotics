# This message provides data for a list of obstacles from a single source

float64 timestamp

string process_name
string process_namespace

string source_topic
float64 source_timestamp

string navpose_frame
nepi_interfaces/NavPose navpose_msg

Obstacle[] obstacles

# Per-cycle depth map segmentation, carried with the obstacle list so the
# image publisher node overlays exactly the maps the obstacles were derived
# from. Both are 32FC1 range images the same size as the source depth map,
# with non-member pixels set to NaN. Empty (0x0) images when the segmentation
# was not produced this cycle.
sensor_msgs/Image depth_map_ground
sensor_msgs/Image depth_map_obstacles
