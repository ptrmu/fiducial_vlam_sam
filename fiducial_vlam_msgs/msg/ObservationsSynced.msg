# A set of image's marker observations

# A time stamp for when these observations were made and the (camera) frame of the base of the camera that made the observations
std_msgs/Header header

# A description of the map environment for these markers that were observed
MapEnvironment map_environment

# The observations
Observations[] observations_synced
