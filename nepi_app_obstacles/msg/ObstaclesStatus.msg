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

# Colourized renders of the per-cycle depth map segmentation, one entry per
# active imaging source. Both lists are built from the same active source order
# as process_status.imaging_source_topics and imaging_pub_topics, so index i of
# all four lists describes the same source. Each topic carries a bgr8 image, not
# range data -- the raw segmentation stays inside the Obstacles message.
string[] ground_image_pub_topics
string[] obstacles_image_pub_topics
