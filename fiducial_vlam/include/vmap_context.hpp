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


  // The following parameters control how vmap_node publishes (3 messages) and subscribes (1 message) to ROS2 messages
  // Subscriptions:
  //  Observations - Only when a map is being built.
  //    messsage: fiducial_vlam_msgs::msg::Observations
  //    topic parameter: psm_sub_observations_topic ("fiducial_observations")
  // Publications:
  //  Marker Map - At a regular interval. Either loaded from a file or as a map is being built from observations
  //    message: fiducial_vlam_msgs::msg::Map
  //    topic parameter: psm_pub_map_topic ("fiducial_map")
  //    frame_id parameter: psm_pub_map_frame_id ("map")
  //  Marker Visualizations - message with flattened cubes for rviz to visualize the marker locations.
  //    message: visualization_msgs::msg::MarkerArray
  //    enable parameter: psm_pub_visuals_enable (true)
  //    topic parameter: psm_pub_visuals_topic ("fiducial_visuals")
  //    frame_id: same as Marker Map
  //  Marker Transformations - publish the transformations to each of the markers
  //    message: tf2_msgs::msg::TFMessage
  //    enable parameter: psm_pub_tfs_enable (true)
  //    topic: "tf"
  //    frame_id: same as Marker Map
  //    child_frame_id parameter: psm_pub_tfs_child_frame_id ("marker_")
  //
#define PSM_ALL_PARAMS \
  PAMA_PARAM(psm_sub_observations_topic, std::string, "/fiducial_observations") /* topic for subscription to fiducial_vlam_msgs::msg::Observations  */\
  PAMA_PARAM(psm_pub_map_topic, std::string, "/fiducial_map") /* topic for publishing map of markers  */\
  PAMA_PARAM(psm_pub_visuals_topic, std::string, "fiducial_visuals") /* topic for publishing rviz visualizations of markers  */\
  \
  PAMA_PARAM(psm_pub_map_frame_id, std::string, "map")    /* frame_id for marker and tf messages - normally "map"  */\
  PAMA_PARAM(psm_pub_tfs_child_frame_id, std::string, "marker_") /* frame_id prefix for each marker in the tf message  */\
  \
  PAMA_PARAM(psm_pub_visuals_enable, bool, true)          /* Enable publishing of the marker visualizations */\
  PAMA_PARAM(psm_pub_tfs_enable, bool, true)              /* Enable publishing of marker tfs */\
  PAMA_PARAM(psm_pub_map_frequency_hz, double, 0.)        /* Hz => rate at which the marker map is published */\
  /* End of list */


  // The following parameters control how marker maps are built from observations
#define BMM_ALL_PARAMS \
  PAMA_PARAM(bmm_algorithm, int, 1)                       /* 0->record observations to file, 1->t_marker0_marker1, 2->isam_betweenfactor  */\
  PAMA_PARAM(bmm_recorded_observations_name, std::string, "observations.yaml") /* topic for publishing map of markers  */                   \
  PAMA_PARAM(bmm_solve_tmm_algorithm, int, 1)             /* 0->cv-SolvePnp+EstimateMAC  */\
  PAMA_PARAM(average_on_space_not_manifold, bool, true)   /* Estimate t_marker0_marker1 in TangentSpace or Manifold space  */\
  PAMA_PARAM(bmm_tmm_try_shonan, bool, false)             /* Use Shonan Rotational Averaging to initialize optimization initial values  */\
  PAMA_PARAM(bmm_tmm_noise_strategy, int, 1)              /* 0->use estimate from samples, 1->use fixed sigmas if estimate is below fixed, 2->use fixed sigma  */\
  PAMA_PARAM(bmm_tmm_fixed_sigma_r, double, 0.1)          /* Fixed r sigma to use for measurement noise model when optimizing.  */\
  PAMA_PARAM(bmm_tmm_fixed_sigma_t, double, 0.3)          /* Fixed t sigma to use for measurement noise model when optimizing. */\
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
