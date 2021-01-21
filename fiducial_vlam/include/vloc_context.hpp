#pragma once

#include <string>

#include "ros2_shared/param_macros.hpp"

namespace rclcpp
{
  class Node;
}

namespace fvlam
{
  class Logger;
}

namespace fiducial_vlam
{
#define VLOC_ALL_PARAMS \
  /* vlocnode flags */\
  PAMA_PARAM(loc_camera_sam_not_cv, bool, false)          /* use gtsam not opencv for localizing the camera */\
  PAMA_PARAM(loc_cmd, std::string, "")                    /* commands to vloc_node (diagnostics, ...) */\
   /* Camera frame -> baselink frame transform */\
  PAMA_PARAM(loc_t_camera_base_x, double, 0.)             /* camera=>baselink transform component */\
  PAMA_PARAM(loc_t_camera_base_y, double, 0.)             /* camera=>baselink transform component */\
  PAMA_PARAM(loc_t_camera_base_z, double, 0)              /* camera=>baselink transform component */\
  PAMA_PARAM(loc_t_camera_base_roll, double, TF2SIMD_HALF_PI) /* camera=>baselink transform component */\
  PAMA_PARAM(loc_t_camera_base_pitch, double, -TF2SIMD_HALF_PI) /* camera=>baselink transform component */\
  PAMA_PARAM(loc_t_camera_base_yaw, double, 0.)           /* camera=>baselink transform component */\
  /* Aruco markers detection parameters - done by opencv */\
  PAMA_PARAM(loc_aruco_dictionary_id, int, 0)             /* Aruco dictionary id for localization markers  */ \
  PAMA_PARAM(loc_corner_refinement_method, int, 2)        /* OpenCV 4.x argument to detect corners. 0 = none, 1 = subpix, 2 = contour, 3 = apriltag */\
  /* Parameters for GTSAM localization techniques */\
  PAMA_PARAM(loc_corner_measurement_sigma, double, 0.5)   /* Noise model in GTSAM for marker corners in the image (sigma in pixels) */\
  PAMA_PARAM(loc_use_marker_covariance, bool, true)       /* When localizing a camera, use the covariance stored in the marker. */\
  /* End of list */

  // The following parameters control how vloc_node publishes messages and subscribes to messages
  // Subscriptions:
  //  Image Raw - The captured image that contains views of fiducial markers.
  //    message: sensor_msgs::msg::Image
  //    topic parameter: psl_sub_image_raw_topic ("image_raw")
  //    subscription quality parameter: psl_sub_image_raw_best_effort_not_reliable (true)
  //  Camera Info - The camera calibration data for the camera that captured the Image Raw frame.
  //    message: sensor_msgs::msg::CameraInfo
  //    topic parameter: psl_sub_camera_info_topic ("camera_info")
  //    subscription quality parameter: psl_sub_camera_info_best_effort_not_reliable (true)
  //  Marker Map - The pose of fiducial markers in a world frame. Published and created by vmap_node.
  //    message: fiducial_vlam_msgs::msg::Map
  //    topic parameter: psl_sub_map_topic ("/fiducial_map")
  // Publications:
  //  Image Marked - A copy of the Image Raw frame with the fiducial markers annotated
  //    message: sensor_msgs::msg::Image
  //    enable parameter: psl_pub_image_marked_enable (true)
  //    topic parameter: psl_pub_image_marked_topic ("image_marked")
  //    frame_id parameter: psl_pub_image_marked_frame_id ("map")
  //  Observations - The marker corner locations found in a single image
  //    message: fiducial_vlam_msgs::msg::Observations
  //    enable parameter: psl_pub_observations_enable (true)
  //    topic parameter: psl_pub_observations_topic ("/fiducial_observations")
  //    frame_id parameter: psl_pub_observations_frame_id ("map")
  //  Camera Pose - The pose of the camera as determined by viewing fiducial markers
  //    message: geometry_msgs::msg::PoseWithCovarianceStamped
  //    enable parameter: psl_pub_camera_pose_enable (true)
  //    topic parameter: psl_pub_camera_pose_topic ("camera_pose")
  //    frame_id parameter: psl_pub_map_frame_id ("map")
  //  Camera Odom - The odometry of the camera as determined by viewing fiducial markers
  //    message: nav_msgs::msg::Odometry
  //    enable parameter: psl_pub_camera_odom_enable (true)
  //    topic parameter: psl_pub_camera_odom_topic ("camera_odom")
  //    frame_id parameter: psl_pub_map_frame_id ("map")
  //  Base Pose - The pose of the Base - a fixed transform from the camera
  //    message: geometry_msgs::msg::PoseWithCovarianceStamped
  //    enable parameter: psl_pub_base_pose_enable (true)
  //    topic parameter: psl_pub_base_pose_topic ("base_pose")
  //    frame_id parameter: psl_pub_map_frame_id ("map")
  //  Base Odom - The odometry of the Base - a fixed transform from the camera
  //    message: nav_msgs::msg::Odometry
  //    enable parameter: psl_pub_base_odom_enable (true)
  //    topic parameter: psl_pub_base_odom_topic ("base_odom")
  //    frame_id parameter: psl_pub_map_frame_id ("map")
  //  Marker Transformations - publish the transformations to each of the markers
  //    message: tf2_msgs::msg::TFMessage
  //    enable parameter:
  //      psl_pub_tf_camera_enable (true) publish tf to camera frame
  //      psl_pub_tf_base_enable (true) publish tf to base frame
  //      psl_pub_tf_camera_per_marker_enable (false)
  //      psl_pub_tf_marker_per_marker_enable (false)
  //    topic: "tf"
  //    frame_id: psl_pub_map_frame_id ("map")
  //    child_frame_id parameter:
  //      psl_pub_tf_camera_child_frame_id ("camera")
  //      psl_pub_tf_base_child_frame_id ("base")
  //      psl_pub_tf_camera_per_marker_child_frame_id ("cpm_")
  //      psl_pub_tf_marker_per_marker_child_frame_id ("mpm_")

#define PSL_ALL_PARAMS \
  /* Subscription topics */\
  PAMA_PARAM(psl_sub_image_raw_topic, std::string, "image_raw") /* topic for subscription to sensor_msgs::msg::Image */\
  PAMA_PARAM(psl_sub_camera_info_topic, std::string, "camera_info") /* topic for subscription to sensor_msgs::msg::CameraInfo associated with the image  */\
  PAMA_PARAM(psl_sub_map_topic, std::string, "/fiducial_map") /* topic for subscription to fiducial_vlam_msgs::msg::Map  */\
 /* Subscription message quality */\
  PAMA_PARAM(psl_sub_image_raw_best_effort_not_reliable, bool, true) /* subscribe to image_ros message with best_effort (gazebo camera) not reliable (tello_ros) */\
  PAMA_PARAM(psl_sub_camera_info_best_effort_not_reliable, bool, true) /* subscribe to camera_info message with best_effort (gazebo camera) not reliable (tello_ros) */\
  /* Messages to publish */\
  PAMA_PARAM(psl_pub_image_marked_enable, bool, true)     /* publish the image_marked at every frame  */\
  PAMA_PARAM(psl_pub_observations_enable, bool, true)     /* publish the observations at every frame  */\
  PAMA_PARAM(psl_pub_camera_pose_enable, bool, true)      /* publish the pose of the camera at every frame  */\
  PAMA_PARAM(psl_pub_camera_odom_enable, bool, true)      /* publish the odometry of the camera at every frame  */\
  PAMA_PARAM(psl_pub_base_pose_enable, bool, true)        /* publish the pose of the base at every frame  */\
  PAMA_PARAM(psl_pub_base_odom_enable, bool, true)        /* publish the odometry of the base at every frame  */\
  PAMA_PARAM(psl_pub_tf_camera_enable, bool, true)        /* publish the tf of the camera at every frame  */\
  PAMA_PARAM(psl_pub_tf_base_enable, bool, true)          /* publish the tf of the base at every frame  */\
  PAMA_PARAM(psl_pub_tf_camera_per_marker_enable, bool, false) /* publish the camera tf as determined by each visible marker  */\
  PAMA_PARAM(psl_pub_tf_marker_per_marker_enable, bool, false) /* publish each marker tf as determined by camera pose and observation  */\
  /* Publish topics */\
  PAMA_PARAM(psl_pub_image_marked_topic, std::string, "image_marked") /* topic for republishing the image with axes added to fiducial markers  */\
  PAMA_PARAM(psl_pub_observations_topic, std::string, "/fiducial_observations") /* topic for publishing fiducial observations  */\
  PAMA_PARAM(psl_pub_camera_pose_topic, std::string, "camera_pose") /* topic for publishing camera pose  */\
  PAMA_PARAM(psl_pub_camera_odom_topic, std::string, "camera_odom") /* topic for publishing camera odometry  */\
  PAMA_PARAM(psl_pub_base_pose_topic, std::string, "base_pose") /* topic for publishing base pose  */\
  PAMA_PARAM(psl_pub_base_odom_topic, std::string, "base_odom") /* topic for publishing base odometry  */\
  /* Frame ids for published messages */\
  PAMA_PARAM(psl_pub_image_marked_frame_id, std::string, "camera") /* frame_id for image_marked message  */\
  PAMA_PARAM(psl_pub_observations_frame_id, std::string, "camera") /* frame_id for observations message  */\
  PAMA_PARAM(psl_pub_map_frame_id, std::string, "map")    /* frame_id for camera pose, odom, and tf messages  */\
  PAMA_PARAM(psl_pub_tf_camera_child_frame_id, std::string, "camera") /* frame_id for the child in the camera tf message  */\
  PAMA_PARAM(psl_pub_tf_base_child_frame_id, std::string, "base")  /* frame_id for the child in the base tf message  */\
  PAMA_PARAM(psl_pub_tf_camera_per_marker_child_frame_id, std::string, "cpm_")  /* frame_id for the child in the camera_per_marker tf messages  */\
  PAMA_PARAM(psl_pub_tf_marker_per_marker_child_frame_id, std::string, "mpm_")  /* frame_id for the child in the marker_per_marker tf messages  */\
  /* End of list */

  struct VlocContext
  {
#undef PAMA_PARAM
#define PAMA_PARAM(n, t, d) PAMA_PARAM_DEFINE(n, t, d)
    PAMA_PARAMS_DEFINE(VLOC_ALL_PARAMS)
  };

  struct VlocDiagnostics
  {
    std::uint64_t sub_camera_info_count_{0};
    std::uint64_t sub_image_raw_count_{0};
    std::uint64_t sub_map_count_{0};
    std::uint64_t empty_observations_count_{0};
    std::uint64_t invalid_t_map_camera_count_{0};
    std::uint64_t pub_observations_count_{0};
    std::uint64_t pub_camera_pose_count_{0};
    std::uint64_t pub_camera_odom_count_{0};
    std::uint64_t pub_base_pose_count_{0};
    std::uint64_t pub_base_odom_count_{0};
    std::uint64_t pub_tf_count_{0};
    std::uint64_t pub_image_marked_count_{0};
    rclcpp::Time start_time_;

    explicit VlocDiagnostics(const rclcpp::Time &start_time) :
      start_time_{start_time}
    {}

    void report(fvlam::Logger &logger, const rclcpp::Time &end_time);
  };
}
