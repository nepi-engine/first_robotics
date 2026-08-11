# This message provides data for a list of obstacles from a single source

float64 timestamp

string process_name
string process_namespace

string source_topic
float64 source_timestamp

string navpose_frame
<<<<<<< HEAD
nepi_interfaces/NavPose navpose_msg
Obstacle[] obstacles

=======
NavPose navpose_msg
nepi_app_obstacles/Obstacles[] obstacles
>>>>>>> e156e30 (**)
