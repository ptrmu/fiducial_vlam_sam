#pragma once

#include <string>

#include "ros2_shared/param_macros.hpp"

namespace fvlam
{
  class Logger;
}

namespace fiducial_vlam
{
#define VDET_ALL_PARAMS \
  /* Aruco markers detection parameters - done by opencv */\
  PAMA_PARAM(det_map_description, std::string, "the map") /* Description of the map environment */ \
  PAMA_PARAM(det_aruco_dictionary_id, int, 0)             /* Aruco dictionary id for localization markers (launch only) */ \
  PAMA_PARAM(det_marker_length, double, 0.2)              /* Length of sides of all the markers */ \
  PAMA_PARAM(det_corner_refinement_method, int, 2)        /* OpenCV 4.x argument to detect corners. 0 = none, 1 = subpix, 2 = contour, 3 = apriltag */\
  /* Camera frame -> baselink frame transform */\
  PAMA_PARAM(det_t_base_camera_x, double, 0.)             /* camera=>baselink transform component */\
  PAMA_PARAM(det_t_base_camera_y, double, 0.)             /* camera=>baselink transform component */\
  PAMA_PARAM(det_t_base_camera_z, double, 0.)             /* camera=>baselink transform component */\
  PAMA_PARAM(det_t_base_camera_roll, double, M_PI_2)      /* camera=>baselink transform component */\
  PAMA_PARAM(det_t_base_camera_pitch, double, -M_PI_2)    /* camera=>baselink transform component */\
  PAMA_PARAM(det_t_base_camera_yaw, double, 0.)           /* camera=>baselink transform component */\
  /* End of list */

  // The following parameters control how vdet_node publishes messages and subscribes to messages
  // Subscriptions:
  //  Image Raw - The captured image that contains views of fiducial markers.
  //    message: sensor_msgs::msg::Image
  //    topic parameter: psd_sub_image_raw_topic ("image_raw")
  //    subscription quality parameter: psd_sub_image_raw_best_effort_not_reliable (true)
  //  Camera Info - The camera calibration data for the camera that captured the Image Raw frame.
  //    message: sensor_msgs::msg::CameraInfo
  //    topic parameter: psd_sub_camera_info_topic ("camera_info")
  //    subscription quality parameter: psd_sub_camera_info_best_effort_not_reliable (true)
  // Publications:
  //  Image Marked - A copy of the Image Raw frame with the fiducial markers annotated
  //    message: sensor_msgs::msg::Image
  //    enable parameter: psd_pub_image_marked_enable (true)
  //    topic parameter: psd_pub_image_marked_topic ("image_marked")
  //    frame_id parameter: psd_pub_image_marked_frame_id (same as image_raw)
  //  Observations - The marker corner locations found in a single image
  //    message: fiducial_vlam_msgs::msg::ObservationsStamped
  //    enable parameter: psd_pub_observations_enable (true)
  //    topic parameter: psd_pub_observations_topic ("observations")
  //    frame_id parameter: psd_pub_observations_frame_id (same as image_raw)

#define PSD_ALL_PARAMS \
  /* Subscription topics */\
  PAMA_PARAM(psd_sub_image_raw_topic, std::string, "image_raw") /* topic for subscription to sensor_msgs::msg::Image (launch only) */\
  PAMA_PARAM(psd_sub_camera_info_topic, std::string, "camera_info") /* topic for subscription to sensor_msgs::msg::CameraInfo associated with the image  (launch only) */\
  /* Subscription message quality */\
  PAMA_PARAM(psd_sub_image_raw_best_effort_not_reliable, bool, true) /* subscribe to image_ros message with best_effort (gazebo camera) not reliable (tello_ros) (launch only) */\
  PAMA_PARAM(psd_sub_camera_info_best_effort_not_reliable, bool, true) /* subscribe to camera_info message with best_effort (gazebo camera) not reliable (tello_ros) (launch only) */\
  /* Messages to publish */\
  PAMA_PARAM(psd_pub_image_marked_enable, bool, true)     /* publish the image_marked at every frame  */\
  PAMA_PARAM(psd_pub_observations_enable, bool, true)     /* publish the observations at every frame  */\
  /* Publish topics */\
  PAMA_PARAM(psd_pub_image_marked_topic, std::string, "image_marked") /* topic for republishing the image with axes added to fiducial markers  */\
  PAMA_PARAM(psd_pub_observations_topic, std::string, "observations") /* topic for publishing fiducial observations  */\
  /* Frame ids for published messages */\
  PAMA_PARAM(psd_pub_image_marked_frame_id, std::string, "") /* frame_id for image_marked message if non-empty otherwise same as image_raw */\
  PAMA_PARAM(psd_pub_observations_frame_id, std::string, "") /* frame_id for observations message if non-empty otherwise same as image_raw */\
  PAMA_PARAM(psd_pub_camera_info_frame_id, std::string, "")  /* frame_id for our camera_info message if non-empty otherwise same as image_raw */\
  /* End of list */

  struct VdetContext
  {
#undef PAMA_PARAM
#define PAMA_PARAM(n, t, d) PAMA_PARAM_DEFINE(n, t, d)
    PAMA_PARAMS_DEFINE(VDET_ALL_PARAMS)
  };

  struct VdetDiagnostics
  {
    std::uint64_t sub_camera_info_count_{0};
    std::uint64_t sub_image_raw_count_{0};
    std::uint64_t empty_observations_count_{0};
    std::uint64_t pub_observations_count_{0};
    std::uint64_t pub_image_marked_count_{0};
    rclcpp::Time start_time_;

    explicit VdetDiagnostics(const rclcpp::Time &start_time) :
      start_time_{start_time}
    {}

    void report(fvlam::Logger &logger, const rclcpp::Time &end_time);
  };
}
