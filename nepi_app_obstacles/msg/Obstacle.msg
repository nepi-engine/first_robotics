# This message provides data for a single obstacle

float32 timestamp
string name
int16 id
string uid
float32 confidence

# Constants to indicate unset fields
int32 INT_FIELD_UNSET=-999
float32 FLOAT_FIELD_UNSET=-999.0


# Pixel Data
int32 xmin_pixel
int32 xmax_pixel
int32 ymin_pixel
int32 ymax_pixel
int32 width_pixels
int32 height_pixels
float32 area_pixels
float32 area_ratio
geometry_msgs/Vector3 vel_pixels

# Position Data ENU Reference Frame
float32 range_m
float32 azimuth_deg
float32 elevation_deg
