#ifndef FIDUCIAL_VLAM_VMAP_CONTEXT_HPP
#define FIDUCIAL_VLAM_VMAP_CONTEXT_HPP

#include "ros2_shared/param_macros.hpp"
#include "transform_with_covariance.hpp"

namespace rclcpp
{
  class Node;
}

namespace fiducial_vlam
{
#define VMAP_ALL_PARAMS \
  PAMA_PARAM(map_save_filename, std::string, "fiducial_marker_locations.yaml") /* name of the file to store the marker map in  */\
  PAMA_PARAM(map_load_filename, std::string, "fiducial_marker_locations.yaml")  /* name of the file to load the marker map from  */\
  PAMA_PARAM(map_init_style, int, 1)                      /* 0->marker id, pose from file, 1->marker id, pose as parameter, 2->camera pose as parameter  */\
  PAMA_PARAM(map_init_id, int, 0)                         /* marker id for map initialization */\
  PAMA_PARAM(map_init_pose_x, double, 2.)                 /* pose component for map initialization */\
  PAMA_PARAM(map_init_pose_y,double, 0.)                  /* pose component for map initialization */\
  PAMA_PARAM(map_init_pose_z, double, 0.)                 /* pose component for map initialization */\
  PAMA_PARAM(map_init_pose_roll, double, TF2SIMD_HALF_PI) /* pose component for map initialization */\
  PAMA_PARAM(map_init_pose_pitch, double, 0.)             /* pose component for map initialization */\
  PAMA_PARAM(map_init_pose_yaw, double, -TF2SIMD_HALF_PI) /* pose component for map initialization */\
  PAMA_PARAM(map_marker_length, double, 0.2100)           /* length of a side of a marker in meters */\
  \
  PAMA_PARAM(map_cmd, std::string, "")                    /* commands to the build_marker_map system  */\
  PAMA_PARAM(map_skip_images, int, 5)                     /* image frames to skip when creating map. 1=>use all frames, 2=>use every other frame, ...  */\
  PAMA_PARAM(map_corner_measurement_sigma, double, 2.0)   /* Noise model in GTSAM for marker corners in the image (sigma in pixels) */\
  PAMA_PARAM(map_compute_on_thread, int, 1)               /* Do heavy-duty computation on a thread. */\
  /* End of list */

#define PSM_ALL_PARAMS \
  PAMA_PARAM(psm_fiducial_map_pub_topic, std::string, "/fiducial_map") /* topic for publishing map of markers  */\
  PAMA_PARAM(psm_fiducial_markers_pub_topic, std::string, "fiducial_markers") /* topic for publishing rviz visualizations of the fiducial markers  */\
  PAMA_PARAM(psm_fiducial_observations_sub_topic, std::string, "/fiducial_observations") /* topic for subscription to fiducial_vlam_msgs::msg::Observations  */\
  \
  PAMA_PARAM(psm_map_frame_id, std::string, "map")        /* frame_id for marker and tf messages - normally "map"  */\
  PAMA_PARAM(psm_marker_prefix_frame_id, std::string, "marker_") /* frame_id for the child in the marker tf message  */\
  \
  PAMA_PARAM(psm_publish_tfs, int, 1)                     /* non-zero => publish the tf of all the known markers  */\
  PAMA_PARAM(psm_publish_marker_visualizations, int, 1)   /* non-zero => publish a shape that represents a marker  */\
  PAMA_PARAM(psm_marker_map_publish_frequency_hz, double, 0.) /* Hz => rate at which the marker map is published */\
  /* End of list */

  struct VmapContext
  {
#undef PAMA_PARAM
#define PAMA_PARAM(n, t, d) PAMA_PARAM_DEFINE(n, t, d)
    PAMA_PARAMS_DEFINE(VMAP_ALL_PARAMS)

    TransformWithCovariance map_init_transform_;
  };
}

#endif //FIDUCIAL_VLAM_VMAP_CONTEXT_HPP
