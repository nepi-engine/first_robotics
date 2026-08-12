# This message details the Obstacles Status Msg

nepi_interfaces/ProcessStatus process_status

string controls_topic

float32 min_range_m
float32 max_range_m

bool has_navpose
string navpose_topic
bool navpose_topic_connected

bool has_color_image
string color_image_topic
bool color_image_topic_connected

bool full_screen_enabled

bool show_sources_enabled
bool show_ground_enabled
bool show_obstacles_enabled

# How transparent each segmentation overlay is drawn on the source image, 0.0
# fully opaque through 1.0 invisible. The image pub node blends at
# (1.0 - transparency), so these read the way the RUI slider labels them.
# Meaningful only while the matching show_*_enabled flag is true.
float32 ground_transparency
float32 obstacles_transparency

# Colourized renders of the per-cycle depth map segmentation, one entry per
# active imaging source. Both lists are built from the same active source order
# as process_status.imaging_source_topics and imaging_pub_topics, so index i of
# all four lists describes the same source. Each topic carries a bgr8 image, not
# range data -- the raw segmentation ships on the obstacles_depth_map topic.
string[] ground_image_pub_topics
string[] obstacles_image_pub_topics
