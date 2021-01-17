
#include <iomanip>
#include <iostream>

#include "cv_bridge/cv_bridge.h"
#include "fiducial_vlam_msgs/msg/map.hpp"
#include "fiducial_vlam_msgs/msg/observations.hpp"
#include "fvlam/camera_info.hpp"
#include "fvlam/localize_camera_interface.hpp"
#include "fvlam/logger.hpp"
#include "fvlam/marker.hpp"
#include "fvlam/observation.hpp"
#include "fvlam/transform3_with_covariance.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "logger_ros2.hpp"
#include "vloc_context.hpp"

namespace fvlam
{
// ==============================================================================
// FiducialMarkerCvContext from method
// ==============================================================================

  template<>
  FiducialMarkerCvContext FiducialMarkerCvContext::from<fiducial_vlam::VlocContext>(
    fiducial_vlam::VlocContext &other)
  {
    FiducialMarkerCvContext cxt{};
    cxt.border_color_red_ = 0;
    cxt.border_color_green_ = 1.0;
    cxt.border_color_blue_ = 0.;
    cxt.aruco_dictionary_id_ = other.loc_aruco_dictionary_id_;
    cxt.cv4_corner_refinement_method_ = other.loc_corner_refinement_method_;
    return cxt;
  }
}

namespace fiducial_vlam
{
// ==============================================================================
// PslContext class
// ==============================================================================

  struct PslContext
  {
#undef PAMA_PARAM
#define PAMA_PARAM(n, t, d) PAMA_PARAM_DEFINE(n, t, d)
    PAMA_PARAMS_DEFINE(PSL_ALL_PARAMS)
  };

// ==============================================================================
// VlocNode class
// ==============================================================================

  class VlocNode : public rclcpp::Node
  {
    LoggerRos2 logger_;
    VlocContext cxt_{};
    PslContext psl_cxt_{};
    std::unique_ptr<fvlam::LocalizeCameraInterface> localize_camera_{};
    std::unique_ptr<fvlam::FiducialMarkerInterface> fiducial_marker_{};
    fvlam::MarkerMap marker_map_{1.0};

    bool current_camera_sam_not_cv_{};

    std::unique_ptr<sensor_msgs::msg::CameraInfo> camera_info_msg_{};
    std_msgs::msg::Header::_stamp_type last_image_stamp_{};

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_marked_{};
    rclcpp::Publisher<fiducial_vlam_msgs::msg::Observations>::SharedPtr pub_observations_{};
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_camera_pose_{};
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_camera_odom_{};
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_base_pose_{};
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_base_odom_{};
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_tf_{};

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_raw_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_camera_info_;
    rclcpp::Subscription<fiducial_vlam_msgs::msg::Map>::SharedPtr sub_map_;

    void validate_parameters()
    {}

    void validate_psl_parameters()
    {}

    void setup_parameters()
    {
#undef PAMA_PARAM
#define PAMA_PARAM(n, t, d) PAMA_PARAM_INIT(n, t, d)
      PAMA_PARAMS_INIT((*this), cxt_, , VLOC_ALL_PARAMS, validate_parameters)
      PAMA_PARAMS_INIT((*this), psl_cxt_, , PSL_ALL_PARAMS, validate_psl_parameters)

#undef PAMA_PARAM
#define PAMA_PARAM(n, t, d) PAMA_PARAM_CHANGED(n, t, d)
      PAMA_PARAMS_CHANGED((*this), cxt_, , VLOC_ALL_PARAMS, validate_parameters, RCLCPP_INFO)
      PAMA_PARAMS_CHANGED((*this), psl_cxt_, , PSL_ALL_PARAMS, validate_psl_parameters, RCLCPP_INFO)

#undef PAMA_PARAM
#define PAMA_PARAM(n, t, d) PAMA_PARAM_LOG(n, t, d)
      PAMA_PARAMS_LOG((*this), cxt_, , VLOC_ALL_PARAMS, RCLCPP_INFO)
      PAMA_PARAMS_LOG((*this), psl_cxt_, , PSL_ALL_PARAMS, RCLCPP_INFO)

#undef PAMA_PARAM
#define PAMA_PARAM(n, t, d) PAMA_PARAM_CHECK_CMDLINE(n, t, d)
      PAMA_PARAMS_CHECK_CMDLINE((*this), , VLOC_ALL_PARAMS PSL_ALL_PARAMS, RCLCPP_ERROR)
    }

    // Solve t_map_camera. Create a LocalizeCameraInterface object if one has not been created yet or
    // the parameter type has changed.
    fvlam::Transform3WithCovariance solve_t_map_camera(const fvlam::Observations &observations,
                                                       const fvlam::CameraInfo &camera_info,
                                                       const fvlam::MarkerMap &map)
    {
      // Check that a LocalizeCameraInterface has been instantiated
      if (!localize_camera_ || cxt_.loc_camera_sam_not_cv_ != current_camera_sam_not_cv_) {
        auto localize_camera_context = fvlam::LocalizeCameraCvContext();
        localize_camera_ = make_localize_camera(localize_camera_context, logger_);
        current_camera_sam_not_cv_ = cxt_.loc_camera_sam_not_cv_;
      }

      return localize_camera_->solve_t_map_camera(observations, camera_info, map);
    }

  public:
    explicit VlocNode(const rclcpp::NodeOptions &options) :
      Node("vloc_node", options),
      logger_{*this}
    {
      // Get parameters from the command line
      setup_parameters();

      // Initialize work objects after parameters have been loaded.
      auto fiducial_marker_context = fvlam::FiducialMarkerCvContext::from(cxt_);
      fiducial_marker_ = make_fiducial_marker(fiducial_marker_context, logger_);

      // ROS publishers. Initialize after parameters have been loaded.
      if (psl_cxt_.psl_pub_image_marked_enable_) {
        pub_image_marked_ = create_publisher<sensor_msgs::msg::Image>(
          psl_cxt_.psl_pub_image_marked_topic_, 16);
      }
      if (psl_cxt_.psl_pub_observations_enable_) {
        pub_observations_ = create_publisher<fiducial_vlam_msgs::msg::Observations>(
          psl_cxt_.psl_pub_observations_topic_, 16);
      }
      if (psl_cxt_.psl_pub_camera_pose_enable_) {
        pub_camera_pose_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          psl_cxt_.psl_pub_camera_pose_topic_, 16);
      }
      if (psl_cxt_.psl_pub_camera_odom_enable_) {
        pub_camera_odom_ = create_publisher<nav_msgs::msg::Odometry>(
          psl_cxt_.psl_pub_camera_odom_topic_, 16);
      }
      if (psl_cxt_.psl_pub_base_pose_enable_) {
        pub_base_pose_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          psl_cxt_.psl_pub_base_pose_topic_, 16);
      }
      if (psl_cxt_.psl_pub_base_odom_enable_) {
        pub_base_odom_ = create_publisher<nav_msgs::msg::Odometry>(
          psl_cxt_.psl_pub_base_odom_topic_, 16);
      }
      if (psl_cxt_.psl_pub_tf_camera_enable_ ||
          psl_cxt_.psl_pub_tf_base_enable_ ||
          psl_cxt_.psl_pub_tf_camera_per_marker_enable_ ||
          psl_cxt_.psl_pub_tf_marker_per_marker_enable_) {
        pub_tf_ = create_publisher<tf2_msgs::msg::TFMessage>(
          "/tf", 16);
      }

      // ROS subscriptions
      auto camera_info_qos = psl_cxt_.psl_sub_camera_info_best_effort_not_reliable_ ?
                             rclcpp::QoS{rclcpp::SensorDataQoS(rclcpp::KeepLast(1))} :
                             rclcpp::QoS{rclcpp::ServicesQoS()};
      sub_camera_info_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        psl_cxt_.psl_sub_camera_info_topic_,
        camera_info_qos,
        [this](sensor_msgs::msg::CameraInfo::UniquePtr msg) -> void
        {
          // Save this message because it will be used for the next image message.
          // Note: we are taking ownership of this object
          camera_info_msg_ = std::move(msg);
        });

      auto image_raw_qos = psl_cxt_.psl_sub_image_raw_best_effort_not_reliable_ ?
                           rclcpp::QoS{rclcpp::SensorDataQoS(rclcpp::KeepLast(1))} :
                           rclcpp::QoS{rclcpp::ServicesQoS()};
      sub_image_raw_ = create_subscription<sensor_msgs::msg::Image>(
        psl_cxt_.psl_sub_image_raw_topic_,
        image_raw_qos,
        [this](sensor_msgs::msg::Image::UniquePtr msg) -> void
        {
#undef SHOW_ADDRESS
#ifdef SHOW_ADDRESS
          static int count = 0;
          RCLCPP_INFO(get_logger(), "%d, %p", count++, reinterpret_cast<std::uintptr_t>(msg.get()));
#endif

          // the stamp to use for all published messages derived from this image message.
          auto stamp{msg->header.stamp};

          if (!camera_info_msg_) {
            logger_.debug() << "Ignore image message because no camera_info has been received yet.";

          } else if ((stamp.nanosec == 0l && stamp.sec == 0l) || stamp == last_image_stamp_) {
            logger_.debug() << "Ignore image message because stamp is zero or the same as the previous.";

            // If we have just done a calibration and want to publish the marked captured
            // images then there is nothing to do with this image so ignore it.
          } else {

            // rviz doesn't like it when time goes backward when a bag is played again.
            // The stamp_msgs_with_current_time_ parameter can help this by replacing the
            // image message time with the current time.
            process_image(std::move(msg), std::move(camera_info_msg_));
          }

          last_image_stamp_ = stamp;
        });

      sub_map_ = create_subscription<fiducial_vlam_msgs::msg::Map>(
        psl_cxt_.psl_sub_map_topic_,
        16,
        [this](const fiducial_vlam_msgs::msg::Map::UniquePtr msg) -> void
        {
          marker_map_ = fvlam::MarkerMap::from(*msg);
        });
//
//      // Timer for publishing map info
//      calibrate_timer_ = create_wall_timer(
//        std::chrono::milliseconds(250),
//        [this]() -> void
//        {
//          calibrate_timer_callback();
//        });

      RCLCPP_INFO(get_logger(), "Using opencv %d.%d.%d", CV_VERSION_MAJOR, CV_VERSION_MINOR, CV_VERSION_REVISION);
      RCLCPP_INFO(get_logger(), "To calibrate camera - set cal_cmd parameter.");
      RCLCPP_INFO(get_logger(), "  cal_cmd capture - Capture an image and add it to the set of calibration images.");
      RCLCPP_INFO(get_logger(),
                  "  cal_cmd calibrate - Take the set of calibration images, do a calibration, and save images, calibration and a report to files.");
      RCLCPP_INFO(get_logger(), "  cal_cmd status - Report on the number of images in the set of calibration images.");
      RCLCPP_INFO(get_logger(), "  cal_cmd load_images - Load the set of calibration images from files.");
      RCLCPP_INFO(get_logger(),
                  "  cal_cmd reset - first time: clear the calibration, second time: clear the set of calibration images.");
      RCLCPP_INFO(get_logger(), "vloc_node ready");

      (void) sub_camera_info_;
      (void) sub_image_raw_;
      (void) sub_map_;
    }

  private:
    void process_image(sensor_msgs::msg::Image::UniquePtr image_msg,
                       sensor_msgs::msg::CameraInfo::UniquePtr camera_info_msg)
    {
      auto stamp = image_msg->header.stamp;
      rclcpp::Time time_stamp{stamp};

      // Convert ROS to OpenCV
      cv_bridge::CvImagePtr gray{cv_bridge::toCvCopy(*image_msg, "mono8")};

      // If we are going to publish an annotated image, make a copy of
      // the original message image. If no annotated image is to be published,
      // then just make an empty image pointer. The routines need to check
      // that the pointer is valid before drawing into it.
      cv_bridge::CvImage color_marked;

      if (psl_cxt_.psl_pub_image_marked_enable_) {

        // The toCvShare only makes ConstCvImage because they don't want
        // to modify the original message data. I want to modify the original
        // data so I create another CvImage that is not const and steal the
        // image data. This seems to work now but may not in the future! (3/29/2020)
        // tracked_object is just a dummy variable, it is not tracking anything.
        std::shared_ptr<void const> tracked_object;
        // const_color_marked points to the image data that is owned by image_msg.
        auto const_color_marked = cv_bridge::toCvShare(*image_msg, tracked_object);
        // mat_with_msg_data also references data that is owned by image_msg.
        cv::Mat mat_with_msg_data = const_color_marked->image; // opencv does not copy the image data on assignment.
        // color_marked now also references data that is owned by image_msg
        color_marked = cv_bridge::CvImage{const_color_marked->header,
                                          const_color_marked->encoding,
                                          mat_with_msg_data};
        // When mat_with_msg_data is destroyed, it does not release the image data because it does not own the data.
        // When const_color_marked is destroyed, it does not release the image data because it does not own the data.
        // tracked_object does not really track anything.
        // When color_marked is destroyed at the end of this method, it will not release the image data because
        //  it does not own the data.
        // The image data will finally get released when the image_msg variable is destroyed because it owns the
        //  image data. image_msg gets destroyed last thing in this method because it is passed by value. But that
        //  is after color_marked is destroyed so there is no chance that the data will be references after being
        //  released.

        // Change image_marked to use the standard parent frame
        image_msg->header.frame_id = psl_cxt_.psl_pub_map_frame_id_;
      }

      // Detect the markers in this image and create a list of
      // observations.
      auto observations = fiducial_marker_->detect_markers(gray->image);

      // Annotate the image_marked with the markers that were found.
      if (psl_cxt_.psl_pub_image_marked_enable_) {
        fiducial_marker_->annotate_image_with_detected_markers(color_marked.image, observations);
      }

      // Publish the observations.
      if (psl_cxt_.psl_pub_observations_enable_ &&
          !observations.observations().empty()) {
        auto msg = fiducial_vlam_msgs::msg::Observations{}
          .set__header(image_msg->header)
          .set__camera_info(*camera_info_msg)
          .set__observations(observations.to<std::vector<fiducial_vlam_msgs::msg::Observation>>());
        pub_observations_->publish(msg);
      }

      // Debugging hint: If the markers in color_marked are not outlined
      // in green, then they haven't been detected. If the markers in
      // color_marked are outlined but they have no axes drawn, then
      // no map message has been received. This could be because vmap_node
      // is not running or has not been able to find the starting node.
      // We need a map and observations before we can publish camera
      // localization information.
      if (!marker_map_.empty() && !observations.observations().empty()) {
        auto camera_info = fvlam::CameraInfo::from(*camera_info_msg);

        // Find the camera pose from the observations.
        auto t_map_camera = solve_t_map_camera(observations, camera_info, marker_map_);

        if (t_map_camera.is_valid()) {

          // If annotated images have been requested, then add the annotations now.
          if (psl_cxt_.psl_pub_image_marked_enable_) {
            for (auto &observation : observations.observations()) {
              auto marker = marker_map_.find_marker_const(observation.id());
              if (marker != nullptr) {
                auto t_camera_marker = t_map_camera.tf().inverse() * marker->t_world_marker().tf();
                fiducial_marker_->annotate_image_with_marker_axis(color_marked.image, t_camera_marker,
                                                                  camera_info, 0.5 * marker_map_.marker_length());
              }
            }
          }

          // Find the transform from the base of the robot to the map. Also include the covariance.
          // Note: the covariance values are with respect to the map frame so both t_map_camera and
          // t_map_base have the same covariance.
          auto t_map_base = fvlam::Transform3WithCovariance{
            t_map_camera.tf() * fvlam::Transform3{
              fvlam::Rotate3::RzRyRx(cxt_.loc_t_camera_base_yaw_,
                                     cxt_.loc_t_camera_base_pitch_,
                                     cxt_.loc_t_camera_base_roll_),
              fvlam::Translate3{cxt_.loc_t_camera_base_x_,
                                cxt_.loc_t_camera_base_y_,
                                cxt_.loc_t_camera_base_z_}},
            t_map_camera.cov()};

          // Header to use with pose, odometry, and tf messages
          auto po_header = image_msg->header;
          po_header.frame_id = psl_cxt_.psl_pub_map_frame_id_;

          // Publish the camera pose/odometry in the map frame
          if (psl_cxt_.psl_pub_camera_pose_enable_) {
            auto msg = geometry_msgs::msg::PoseWithCovarianceStamped{}
              .set__header(po_header)
              .set__pose(t_map_camera.to<geometry_msgs::msg::PoseWithCovariance>());
            pub_camera_pose_->publish(msg);
          }
          if (psl_cxt_.psl_pub_camera_odom_enable_) {
            auto msg = nav_msgs::msg::Odometry{}
              .set__header(po_header)
              .set__child_frame_id(psl_cxt_.psl_pub_tf_camera_child_frame_id_)
              .set__pose(t_map_camera.to<geometry_msgs::msg::PoseWithCovariance>());
            pub_camera_odom_->publish(msg);
          }

          // Publish the base pose/odometry in the map frame
          if (psl_cxt_.psl_pub_base_pose_enable_) {
            auto msg = geometry_msgs::msg::PoseWithCovarianceStamped{}
              .set__header(po_header)
              .set__pose(t_map_base.to<geometry_msgs::msg::PoseWithCovariance>());
            pub_base_pose_->publish(msg);
          }
          if (psl_cxt_.psl_pub_base_odom_enable_) {
            auto msg = nav_msgs::msg::Odometry{}
              .set__header(po_header)
              .set__child_frame_id(psl_cxt_.psl_pub_tf_base_child_frame_id_)
              .set__pose(t_map_base.to<geometry_msgs::msg::PoseWithCovariance>());
            pub_base_odom_->publish(msg);
          }

          // Publish all tfs in one message
          tf2_msgs::msg::TFMessage tfs_msg;

          // Publish the camera's tf
          if (psl_cxt_.psl_pub_tf_camera_enable_) {
            auto msg = geometry_msgs::msg::TransformStamped{}
              .set__header(po_header)
              .set__child_frame_id(psl_cxt_.psl_pub_tf_camera_child_frame_id_)
              .set__transform(t_map_camera.tf().to<geometry_msgs::msg::Transform>());
            tfs_msg.transforms.emplace_back(msg);
          }

          // Publish the base's tf
          if (psl_cxt_.psl_pub_tf_base_enable_) {
            auto msg = geometry_msgs::msg::TransformStamped{}
              .set__header(po_header)
              .set__child_frame_id(psl_cxt_.psl_pub_tf_base_child_frame_id_)
              .set__transform(t_map_base.tf().to<geometry_msgs::msg::Transform>());
            tfs_msg.transforms.emplace_back(msg);
          }

          // if requested, publish the camera tf as determined from each marker in world/map coordinates.
          if (psl_cxt_.psl_pub_tf_camera_per_marker_enable_) {
            for (auto &observation : observations.observations()) {
              auto t_marker_camera = localize_camera_->solve_t_camera_marker(
                observation, camera_info, marker_map_.marker_length()).tf().inverse();
              auto marker = marker_map_.find_marker_const(observation.id());
              if (marker != nullptr) {
                auto t_map_camera_n = marker->t_world_marker().tf() * t_marker_camera;
                auto msg = geometry_msgs::msg::TransformStamped{}
                  .set__header(po_header)
                  .set__child_frame_id(compose_frame_id(psl_cxt_.psl_pub_tf_camera_per_marker_child_frame_id_,
                                                        observation.id()))
                  .set__transform(t_map_camera_n.to<geometry_msgs::msg::Transform>());
                tfs_msg.transforms.emplace_back(msg);
              }
            }
          }

          // if requested, publish the marker tf as determined from the camera location and the observation.
          if (psl_cxt_.psl_pub_tf_marker_per_marker_enable_) {
            for (auto &observation : observations.observations()) {
              auto t_camera_marker = localize_camera_->solve_t_camera_marker(
                observation, camera_info, marker_map_.marker_length()).tf();
              auto t_map_marker_n = t_map_camera.tf() * t_camera_marker;
              auto msg = geometry_msgs::msg::TransformStamped{}
                .set__header(po_header)
                .set__child_frame_id(compose_frame_id(psl_cxt_.psl_pub_tf_marker_per_marker_child_frame_id_,
                                                      observation.id()))
                .set__transform(t_map_marker_n.to<geometry_msgs::msg::Transform>());
              tfs_msg.transforms.emplace_back(msg);
            }
          }

          // If there are any transforms to publish, then publish them.
          if (!tfs_msg.transforms.empty()) {
            pub_tf_->publish(tfs_msg);
          }
        }
      }

      // Publish an annotated image if requested. Even if there is no map.
      if (psl_cxt_.psl_pub_image_marked_enable_) {
        // The marking has been happening on the original message.
        // Republish it now.
        pub_image_marked_->publish(std::move(image_msg));
      }
    }

    static std::string compose_frame_id(std::string &prefix, uint64_t id)
    {
      std::ostringstream oss;
      oss << prefix
          << std::setfill('0') << std::setw(3)
          << id;
      return oss.str();
    }


#if 0
    void calibrate_timer_callback()
    {
      rclcpp::Time time_now{now()};

      // Figure out if there is a command to process.
      if (!cal_cxt_.cal_cmd_.empty()) {
        std::string cmd{cal_cxt_.cal_cmd_};

        // Reset the cmd_string in preparation for the next command.
        PAMA_SET_PARAM((*this), cal_cxt_, "", cal_cmd, "");

        // If we are not in calibrate mode, then don't send the command.
        if (true) {
          RCLCPP_ERROR(get_logger(), "Cannot execute cal_cmd when not in calibrate mode");

        } else {
          auto ret_str = cc_pi_->cal_cmd(cmd, time_now);
          if (!ret_str.empty()) {
            RCLCPP_INFO(get_logger(), "cal_cmd '%s' response:\n%s", cmd.c_str(), ret_str.c_str());
          }
        }
      }

      // Give the camera calibrator process some background time
      if (false) {
        auto ret_str = cc_pi_->on_timer(time_now);
        if (!ret_str.empty()) {
          RCLCPP_INFO(get_logger(), "cal_on_timer response:\n%s", ret_str.c_str());
        }
      }

      // If we have just done a calibration and want to publish the marked captured
      // images then do it.
//      if (publish_captured_image_marked()) {
//        cv::Mat captured_image_marked;
//        cc_pi_->get_captured_image_marked(time_now, captured_image_marked);
//        if (captured_image_marked.dims != 0) {
//
//          // build up an image message and publish it.
//          std_msgs::msg::Header header;
//          header.stamp = time_now;
//          header.frame_id = "captured_image_marked";
//          cv_bridge::CvImage cv_image{header, "bgr8", captured_image_marked};
//          pub_image_marked_->publish(*cv_image.toImageMsg());
//        }
//      }
    }
#endif
  };

  std::shared_ptr<rclcpp::Node> vloc_node_factory(const rclcpp::NodeOptions &options)
  {
    return std::shared_ptr<rclcpp::Node>(new VlocNode(options));
  }
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(fiducial_vlam::VlocNode)
