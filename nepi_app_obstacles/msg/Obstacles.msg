# This message provides data for a list of obstacles from a single source

float64 timestamp

string process_name
string process_namespace

string source_topic
float64 source_timestamp

string navpose_frame
nepi_interfaces/NavPose navpose_msg

Obstacle[] obstacles

# The per-cycle depth map segmentation is NOT carried here. It ships as a
# separate ObstaclesDepthMap message on the sibling obstacles_depth_map topic,
# so consumers that only need the obstacle list never pay for two full-size
# range images per cycle.
