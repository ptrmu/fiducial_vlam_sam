# A Camera Calibration

# The name of the transformation frame that defines the image of this camera.
# This frame has z pointing in the direction of the camera lens with x going
# across the image and y going down the image.
string camera_frame_id

# Dimensions of the image
uint32 width
uint32 height

# The intrinsic matrix of the pinhole camera model for this camera.
# The naming here is following the openCV notation convention
float64 fx
float64 fy
float64 cx
float64 cy

# Distortion coefficients
float64 k1
float64 k2
float64 p1
float64 p2
float64 k3

# Transform from camera image frame to cambase frame.
geometry_msgs/Transform t_cambase_camera
