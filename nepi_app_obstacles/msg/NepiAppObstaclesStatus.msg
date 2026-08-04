# This message details the Obstacles App Status Msg

# App enabled state
bool enabled
# Available options (published so the RUI menu always matches the node)
string[] options
# Currently selected option
string selected_option
# Current value setting
float32 value
# Selected depth map source topic, or "None" when nothing is connected
string depth_map_topic
# Depth map image topic belonging to the depth map selected above. Derived from
# that depth map's DepthMapStatus, never selected directly: image_topic when it
# resolves under the selected depth map namespace, otherwise the depth map image
# DepthMapIF publishes at <depth_map>/depth_map_image when img_pub_enabled is
# set. "None" when nothing is connected or the selected depth map reports no
# associated image. Never the color image -- see obstacles_app_node.py
# getDepthMapImageTopic()
string depth_map_image_topic
# Selected color image source topic, or "None" when nothing is connected
string image_topic
# Selected NavPose source topic, or "None" when nothing is connected
string navpose_topic
