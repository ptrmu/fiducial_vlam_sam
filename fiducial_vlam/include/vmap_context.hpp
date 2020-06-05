#ifndef FIDUCIAL_VLAM_VMAP_CONTEXT_HPP
#define FIDUCIAL_VLAM_VMAP_CONTEXT_HPP

#include "ros2_shared/context_macros.hpp"
#include "transform_with_covariance.hpp"

namespace rclcpp
{
  class Node;
}

namespace fiducial_vlam
{
#define VMAP_ALL_PUBSUB_PARAMS \
  CXT_MACRO_MEMBER(mem_fiducial_map_pub_topic, std::string, "/fiducial_map") /* topic for publishing map of markers  */\
  CXT_MACRO_MEMBER(mem_fiducial_markers_pub_topic, std::string, "fiducial_markers") /* topic for publishing rviz visualizations of the fiducial markers  */\
  CXT_MACRO_MEMBER(mem_fiducial_observations_sub_topic, std::string, "/fiducial_observations") /* topic for subscription to fiducial_vlam_msgs::msg::Observations  */\
  \
  CXT_MACRO_MEMBER(mem_map_frame_id, std::string, "map")            /* frame_id for marker and tf messages - normally "map"  */\
  CXT_MACRO_MEMBER(mem_marker_prefix_frame_id, std::string, "marker_") /* frame_id for the child in the marker tf message  */\
  \
  CXT_MACRO_MEMBER(mem_publish_tfs, int, 1)                         /* non-zero => publish the tf of all the known markers  */\
  CXT_MACRO_MEMBER(mem_publish_marker_visualizations, int, 1)       /* non-zero => publish a shape that represents a marker  */\
  CXT_MACRO_MEMBER(mem_marker_map_publish_frequency_hz, double, 0.) /* Hz => rate at which the marker map is published */\
  /* End of list */


#define VMAP_ALL_MAP_PARAMS \
  CXT_MACRO_MEMBER(marker_map_save_full_filename, std::string, "fiducial_marker_locations.yaml") /* name of the file to store the marker map in  */\
  CXT_MACRO_MEMBER(marker_map_load_full_filename, std::string, "")  /* name of the file to load the marker map from  */\
  CXT_MACRO_MEMBER(map_init_style, int, 1)                          /* 0->marker id, pose from file, 1->marker id, pose as parameter, 2->camera pose as parameter  */\
  CXT_MACRO_MEMBER(map_init_id, int, 0)                             /* marker id for map initialization */\
  CXT_MACRO_MEMBER(map_init_pose_x, double, 2.)                     /* pose component for map initialization */\
  CXT_MACRO_MEMBER(map_init_pose_y,double, 0.)                      /* pose component for map initialization */\
  CXT_MACRO_MEMBER(map_init_pose_z, double, 0.)                     /* pose component for map initialization */\
  CXT_MACRO_MEMBER(map_init_pose_roll, double, TF2SIMD_HALF_PI)     /* pose component for map initialization */\
  CXT_MACRO_MEMBER(map_init_pose_pitch, double, 0.)                 /* pose component for map initialization */\
  CXT_MACRO_MEMBER(map_init_pose_yaw, double, -TF2SIMD_HALF_PI)     /* pose component for map initialization */\
  CXT_MACRO_MEMBER(marker_length, double, 0.1775)                   /* length of a side of a marker in meters */\
  \
  CXT_MACRO_MEMBER(build_marker_map_cmd, std::string, "")           /* commands to the build_marker_map system  */\
  CXT_MACRO_MEMBER(build_marker_map_skip_images, int, 0)            /* image frames to skip when creating map. 1=>use all frames, 2=>use every other frame, ...  */\
  /* End of list */


#define VMAP_ALL_PARAMS VMAP_ALL_PUBSUB_PARAMS VMAP_ALL_MAP_PARAMS

  struct VmapContext
  {
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)
    VMAP_ALL_PARAMS

    TransformWithCovariance map_init_transform_;
    int timer_period_milliseconds_;
  };
}

#endif //FIDUCIAL_VLAM_VMAP_CONTEXT_HPP
