#pragma once

#include <string>

#include "ros2_shared/param_macros.hpp"

namespace fvlam
{
  class Logger;
}

namespace fiducial_vlam
{

  // The following parameters control how vdet_node publishes messages and subscribes to messages
  // Subscriptions:
  //  Image Raw - The captured image that contains views of fiducial markers.
  //    message: sensor_msgs::msg::Image
  //    topic parameter: det_sub_image_raw_topic ("image_raw")
  //    subscription quality parameter: det_sub_image_raw_best_effort_not_reliable (true)
  //  Camera Info - The camera calibration data for the camera that captured the Image Raw frame.
  //    message: sensor_msgs::msg::CameraInfo
  //    topic parameter: det_sub_camera_info_topic ("camera_info")
  //    subscription quality parameter: det_sub_camera_info_best_effort_not_reliable (true)
  // Publications:
  //  Image Marked - A copy of the Image Raw frame with the fiducial markers annotated
  //    message: sensor_msgs::msg::Image
  //    enable parameter: det_pub_image_marked_enable (true)
  //    topic parameter: det_pub_image_marked_topic ("image_marked")
  //    frame_id parameter: det_pub_image_marked_frame_id (same as image_raw)
  //  Observations - The marker corner locations found in a single image
  //    message: fiducial_vlam_msgs::msg::ObservationsSynced
  //    enable parameter: det_pub_observations_enable (true)
  //    topic parameter: det_pub_observations_topic ("observations")
  //    frame_id parameter:
  //      det_pub_imager_frame_id - the imager frame_id
  //      det_pub_camera_frame_id - the camera frame_id

  #define VDET_ALL_PARAMS \
  /* Aruco markers detection parameters - done by opencv */\
  PAMA_PARAM(det_map_description, std::string, "the map") /* Description of the map environment */ \
  PAMA_PARAM(det_aruco_dictionary_id, int, 0)             /* Aruco dictionary id for localization markers (launch only) */ \
  PAMA_PARAM(det_marker_length, double, 0.2)              /* Length of sides of all the markers */ \
  PAMA_PARAM(det_corner_refinement_method, int, 2)        /* OpenCV 4.x argument to detect corners. 0 = none, 1 = subpix, 2 = contour, 3 = apriltag */\
  /* One or more imagers are components of a camera. This is the transform from this imager to the camera */\
  PAMA_PARAM(det_t_camera_imager_x, double, 0.)          /* imager->camera transform component */\
  PAMA_PARAM(det_t_camera_imager_y, double, 0.)          /* imager->camera transform component */\
  PAMA_PARAM(det_t_camera_imager_z, double, 0.)          /* imager->camera transform component */\
  PAMA_PARAM(det_t_camera_imager_roll, double, M_PI_2)   /* imager->camera transform component */\
  PAMA_PARAM(det_t_camera_imager_pitch, double, -M_PI_2) /* imager->camera transform component */\
  PAMA_PARAM(det_t_camera_imager_yaw, double, 0.)        /* imager->camera transform component */\
  /* Subscription topics */\
  PAMA_PARAM(det_sub_image_raw_topic, std::string, "image_raw") /* topic for subscription to sensor_msgs::msg::Image (launch only) */\
  PAMA_PARAM(det_sub_camera_info_topic, std::string, "camera_info") /* topic for subscription to sensor_msgs::msg::CameraInfo associated with the image  (launch only) */\
  /* Subscription message quality */\
  PAMA_PARAM(det_sub_image_raw_best_effort_not_reliable, bool, true) /* subscribe to image_ros message with best_effort (gazebo camera) not reliable (tello_ros) (launch only) */\
  PAMA_PARAM(det_sub_camera_info_best_effort_not_reliable, bool, true) /* subscribe to camera_info message with best_effort (gazebo camera) not reliable (tello_ros) (launch only) */\
  /* Messages to publish */\
  PAMA_PARAM(det_pub_image_marked_enable, bool, true)     /* publish the image_marked at every frame  */\
  PAMA_PARAM(det_pub_observations_enable, bool, true)     /* publish the observations at every frame  */\
  /* Publish topics */\
  PAMA_PARAM(det_pub_image_marked_topic, std::string, "image_marked") /* topic for republishing the image with borders around the fiducial markers  */\
  PAMA_PARAM(det_pub_observations_topic, std::string, "observations") /* topic for publishing observations from this imager  */\
  /* Frame ids for published messages */\
  PAMA_PARAM(det_pub_image_marked_frame_id, std::string, "") /* frame_id for image_marked message if non-empty otherwise same as image_raw */\
  PAMA_PARAM(det_pub_imager_frame_id, std::string, "imager_frame") /* frame_id for CameraInfo message if non-empty otherwise same as image_raw */\
  PAMA_PARAM(det_pub_camera_frame_id, std::string, "")  /* frame_id for ObservationSynced message if non-empty otherwise same as image_raw */\
  /* End of list */

  struct VdetContext
  {
#undef PAMA_PARAM
#define PAMA_PARAM(n, t, d) PAMA_PARAM_DEFINE(n, t, d)
    PAMA_PARAMS_DEFINE(VDET_ALL_PARAMS)
  };
}
