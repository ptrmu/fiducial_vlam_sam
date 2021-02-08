# A fiducial marker.

# The marker id
uint64 id

# A flag that this is a fixed marker that must not be moved.
bool is_fixed

# The location of the marker in map/world frame
geometry_msgs/PoseWithCovariance t_map_marker
