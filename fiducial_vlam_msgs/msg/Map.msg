# A list of the markers in the environment

std_msgs/Header header

# Length in meters of a side of all markers
float64 marker_length

# Style of map
uint8 POSE=0        # Marker Pose only
uint8 COVARIANCE=1  # Marker Pose and Covariance
uint8 CORNERS=2     # Marker Pose and Covariance and Corner Points and Covariance
uint8 map_style

# id and pose of the markers
int32[] fixed_flags
int32[] ids
geometry_msgs/PoseWithCovariance[] poses
