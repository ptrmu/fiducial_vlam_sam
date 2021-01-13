#pragma once
#include <string>

#include "ros2_shared/param_macros.hpp"
#include "transform_with_covariance.hpp"

namespace rclcpp
{
  class Node;
}

namespace fiducial_vlam
{
#define VLOC_ALL_PARAMS \
  /* vlocnode flags */\
  PAMA_PARAM(loc_calibrate_not_localize, int, 0)          /* calibrate camera instead of localize camera */ \
  PAMA_PARAM(loc_camera_sam_not_cv, int, 0)               /* use gtsam not opencv for localizing the camera */\
  PAMA_PARAM(loc_corner_measurement_sigma, double, 2.0)   /* Noise model in GTSAM for marker corners in the image (sigma in pixels) */\
   /* Camera frame -> baselink frame transform */\
  PAMA_PARAM(loc_t_camera_base_x, double, 0.)             /* camera=>baselink transform component */\
  PAMA_PARAM(loc_t_camera_base_y, double, 0.)             /* camera=>baselink transform component */\
  PAMA_PARAM(loc_t_camera_base_z, double, -0.035)         /* camera=>baselink transform component */\
  PAMA_PARAM(loc_t_camera_base_roll, double, TF2SIMD_HALF_PI) /* camera=>baselink transform component */\
  PAMA_PARAM(loc_t_camera_base_pitch, double, -TF2SIMD_HALF_PI) /* camera=>baselink transform component */\
  PAMA_PARAM(loc_t_camera_base_yaw, double, 0.)           /* camera=>baselink transform component */\
  /* Corner Kalman Filters */\
  PAMA_PARAM(loc_corner_filter_process_std, float, 0.)    /* Process Noise Std Dev for Kalman filtering corners */\
  PAMA_PARAM(loc_corner_filter_measure_std, float, 0.)    /* Measurement Noise Std Dev for Kalman filtering corners */\
  PAMA_PARAM(loc_corner_filter_max_skipped, int, 3)       /* Max number of frames that to predict_only corner locations */\
  /* Aruco markers */\
  PAMA_PARAM(loc_aruco_dictionary_id, int, 0)             /* Aruco dictionary id for localization markers  */ \
  PAMA_PARAM(loc_cv3_do_corner_refinement, int, 1)        /* OpenCV 3.x argument to detect corners. 0 = false, 1 = true */\
  PAMA_PARAM(loc_cv4_corner_refinement_method, int, 2)    /* OpenCV 4.x argument to detect corners. 0 = none, 1 = subpix, 2 = contour, 3 = apriltag */\
  /* End of list */

#define PSL_ALL_PARAMS \
 /* Mssages for vLocnode options */\
  PAMA_PARAM(psl_stamp_msgs_with_current_time, int, 0)    /* non-zero => debug mode, helpful for dealing with rviz when playing bags.  */\
  PAMA_PARAM(psl_sub_camera_info_best_effort_not_reliable, int, 1) /* subscribe to camera_info message with best_effort (gazebo camera) not reliable (tello_ros) */\
  PAMA_PARAM(psl_sub_image_raw_best_effort_not_reliable, int, 1) /* subscribe to image_ros message with best_effort (gazebo camera) not reliable (tello_ros) */\
  /* Subscription topics */\
  PAMA_PARAM(psl_fiducial_map_sub_topic, std::string, "/fiducial_map")/* topic for subscription to fiducial_vlam_msgs::msg::Map  */\
  PAMA_PARAM(psl_camera_info_sub_topic, std::string, "camera_info")   /* topic for subscription to sensor_msgs::msg::CameraInfo associated with the image  */\
  PAMA_PARAM(psl_image_raw_sub_topic, std::string, "image_raw")       /* topic for subscription to sensor_msgs::msg::Image */\
  /* Messages to publish */\
  PAMA_PARAM(psl_publish_camera_pose, int, 1)             /* non-zero => publish the pose of the camera at every frame  */\
  PAMA_PARAM(psl_publish_base_pose, int, 1)               /* non-zero => publish the pose of the base at every frame  */\
  PAMA_PARAM(psl_publish_tfs, int, 1)                     /* non-zero => publish the tf of the camera at every frame  */\
  PAMA_PARAM(psl_publish_camera_tf_per_marker, int, 0)    /* non-zero => publish the camera tf/pose as determined by each visible marker  */\
  PAMA_PARAM(psl_publish_marker_tf_per_marker, int, 0)    /* non-zero => publish the marker tf/pose as determined by camera pose and observation  */\
  PAMA_PARAM(psl_publish_camera_odom, int, 1)             /* non-zero => publish the odometry of the camera at every frame  */\
  PAMA_PARAM(psl_publish_base_odom, int, 1)               /* non-zero => publish the odometry of the base at every frame  */\
  PAMA_PARAM(psl_publish_image_marked, int, 1)            /* non-zero => publish the image_marked at every frame  */\
  /* Publish topics */\
  PAMA_PARAM(psl_fiducial_observations_pub_topic, std::string, "/fiducial_observations")  /* topic for publishing fiducial observations  */\
  PAMA_PARAM(psl_camera_pose_pub_topic, std::string, "camera_pose")    /* topic for publishing camera pose  */\
  PAMA_PARAM(psl_base_pose_pub_topic, std::string, "base_pose")        /* topic for publishing base pose  */\
  PAMA_PARAM(psl_camera_odometry_pub_topic, std::string, "camera_odom")/* topic for publishing camera odometry  */\
  PAMA_PARAM(psl_base_odometry_pub_topic, std::string, "base_odom")    /* topic for publishing base odometry  */\
  PAMA_PARAM(psl_image_marked_pub_topic, std::string, "image_marked")  /* topic for republishing the image with axes added to fiducial markers  */\
  /* Frame ids for published messages */\
  PAMA_PARAM(psl_map_frame_id, std::string, "map")        /* frame_id for camera pose and tf messages - normally "map"  */\
  PAMA_PARAM(psl_camera_frame_id, std::string, "camera")  /* frame_id for the child in the camera tf message  */\
  PAMA_PARAM(psl_base_frame_id, std::string, "base_link") /* frame_id for the child in the base_link tf message  */\
  /* End of list */

  struct VlocContext
  {
#undef PAMA_PARAM
#define PAMA_PARAM(n, t, d) PAMA_PARAM_DEFINE(n, t, d)
    PAMA_PARAMS_DEFINE(VLOC_ALL_PARAMS)

    TransformWithCovariance loc_t_camera_base_;
  };
}
