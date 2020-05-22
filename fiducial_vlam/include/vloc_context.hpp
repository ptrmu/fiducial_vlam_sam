#ifndef FIDUCIAL_VLAM_VLOC_CONTEXT_HPP
#define FIDUCIAL_VLAM_VLOC_CONTEXT_HPP

#include <string>

#include "ros2_shared/context_macros.hpp"
#include "transform_with_covariance.hpp"

namespace rclcpp
{
  class Node;
}

namespace fiducial_vlam
{
#define VLOC_ALL_PARAMS \
  /* vlocnode flags */\
  CXT_MACRO_MEMBER(loc_calibrate_not_loocalize, int, 1)         /* calibrate camera instead of localize camera */ \
  CXT_MACRO_MEMBER(loc_camera_sam_not_cv, int, 0)               /* use gtsam not opencv for localizing the camera */\
  CXT_MACRO_MEMBER(loc_corner_measurement_sigma, double, 2.0)   /* Noise model in GTSAM for marker corners in the image (sigma in pixels) */\
  /* MEssages for vLocnode options */\
  CXT_MACRO_MEMBER(mel_stamp_msgs_with_current_time, int, 0)    /* non-zero => debug mode, helpful for dealing with rviz when playing bags.  */\
  CXT_MACRO_MEMBER(mel_sub_camera_info_best_effort_not_reliable, int, 1) /* subscribe to camera_info message with best_effort (gazebo camera) not reliable (tello_ros) */\
  /* Subscription topics */\
  CXT_MACRO_MEMBER(mel_fiducial_map_sub_topic, std::string, "/fiducial_map")/* topic for subscription to fiducial_vlam_msgs::msg::Map  */\
  CXT_MACRO_MEMBER(mel_camera_info_sub_topic, std::string, "camera_info")   /* topic for subscription to sensor_msgs::msg::CameraInfo associated with the image  */\
  CXT_MACRO_MEMBER(mel_image_raw_sub_topic, std::string, "image_raw")       /* topic for subscription to sensor_msgs::msg::Image */\
  /* Messages to publish */\
  CXT_MACRO_MEMBER(mel_publish_camera_pose, int, 1)             /* non-zero => publish the pose of the camera at every frame  */\
  CXT_MACRO_MEMBER(mel_publish_base_pose, int, 1)               /* non-zero => publish the pose of the base at every frame  */\
  CXT_MACRO_MEMBER(mel_publish_tfs, int, 1)                     /* non-zero => publish the tf of the camera at every frame  */\
  CXT_MACRO_MEMBER(mel_publish_tfs_per_marker, int, 0)          /* non-zero => publish the camera tf/pose as determined by each visible marker  */\
  CXT_MACRO_MEMBER(mel_publish_camera_odom, int, 1)             /* non-zero => publish the odometry of the camera at every frame  */\
  CXT_MACRO_MEMBER(mel_publish_base_odom, int, 1)               /* non-zero => publish the odometry of the base at every frame  */\
  CXT_MACRO_MEMBER(mel_publish_image_marked, int, 1)            /* non-zero => publish the image_marked at every frame  */\
  /* Publish topics */\
  CXT_MACRO_MEMBER(fiducial_observations_pub_topic, std::string, "/fiducial_observations")  /* topic for publishing fiducial observations  */\
  CXT_MACRO_MEMBER(camera_pose_pub_topic, std::string, "camera_pose")       /* topic for publishing camera pose  */\
  CXT_MACRO_MEMBER(base_pose_pub_topic, std::string, "base_pose")           /* topic for publishing base pose  */\
  CXT_MACRO_MEMBER(camera_odometry_pub_topic, std::string, "camera_odom")   /* topic for publishing camera odometry  */\
  CXT_MACRO_MEMBER(base_odometry_pub_topic, std::string, "base_odom")       /* topic for publishing base odometry  */\
  CXT_MACRO_MEMBER(image_marked_pub_topic, std::string, "image_marked")     /* topic for republishing the image with axes added to fiducial markers  */\
  /* Frame ids for published messages */\
  CXT_MACRO_MEMBER(map_frame_id, std::string, "map")            /* frame_id for camera pose and tf messages - normally "map"  */\
  CXT_MACRO_MEMBER(camera_frame_id, std::string, "camera")      /* frame_id for the child in the camera tf message  */\
  CXT_MACRO_MEMBER(base_frame_id, std::string, "base_link")     /* frame_id for the child in the base_link tf message  */\
  /* Camera frame -> baselink frame transform */\
  CXT_MACRO_MEMBER(t_camera_base_x, double, 0.)                 /* camera=>baselink transform component */\
  CXT_MACRO_MEMBER(t_camera_base_y, double, 0.)                 /* camera=>baselink transform component */\
  CXT_MACRO_MEMBER(t_camera_base_z, double, -0.035)             /* camera=>baselink transform component */\
  CXT_MACRO_MEMBER(t_camera_base_roll, double, TF2SIMD_HALF_PI) /* camera=>baselink transform component */\
  CXT_MACRO_MEMBER(t_camera_base_pitch, double, -TF2SIMD_HALF_PI) /* camera=>baselink transform component */\
  CXT_MACRO_MEMBER(t_camera_base_yaw, double, 0.)               /* camera=>baselink transform component */\
  /* Corner Kalman Filters */\
  CXT_MACRO_MEMBER(loc_corner_filter_process_std, float, 0.)    /* Process Noise Std Dev for Kalman filtering corners */\
  CXT_MACRO_MEMBER(loc_corner_filter_measure_std, float, 0.)    /* Measurement Noise Std Dev for Kalman filtering corners */\
  CXT_MACRO_MEMBER(loc_corner_filter_max_skipped, int, 3)       /* Max number of frames that to predict_only corner locations */\
  /* Aruco markers */\
  CXT_MACRO_MEMBER(loc_aruco_dictionary_id, int, 10)            /* Aruco dictionary id for localization markers  */ \
  CXT_MACRO_MEMBER(loc_cv3_do_corner_refinement, int, 1)        /* OpenCV 3.x argument to detect corners. 0 = false, 1 = true */\
  CXT_MACRO_MEMBER(loc_cv4_corner_refinement_method, int, 2)    /* OpenCV 4.x argument to detect corners. 0 = none, 1 = subpix, 2 = contour, 3 = apriltag */\
  /* End of list */

  struct VlocContext
  {
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)
    VLOC_ALL_PARAMS

    TransformWithCovariance t_camera_base_;
  };
}

#endif //FIDUCIAL_VLAM_VLOC_CONTEXT_HPP
