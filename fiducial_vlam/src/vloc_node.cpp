
#include <iomanip>
#include <iostream>

//#define ENABLE_TIMING

#include "cv_bridge/cv_bridge.h"
#include "fiducial_vlam/fiducial_vlam.hpp"
#include "fiducial_vlam_msgs/msg/map.hpp"
#include "fiducial_vlam_msgs/msg/observations.hpp"
#include "fiducial_vlam_msgs/msg/observations_stamped.hpp"
#include "fvlam/camera_info.hpp"
#include "fvlam/localize_camera_interface.hpp"
#include "fvlam/logger.hpp"
#include "fvlam/marker.hpp"
#include "fvlam/observation.hpp"
#include "fvlam/transform3_with_covariance.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include <gtsam/base/timing.h>
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "logger_ros2.hpp"
#include "observation_maker.hpp"
#include "vdet_context.hpp"
#include "vloc_context.hpp"

namespace fvlam
{
// ==============================================================================
// LocalizeCameraCvContext from method
// ==============================================================================

  template<>
  LocalizeCameraCvContext LocalizeCameraCvContext::from<fiducial_vlam::VlocContext>(
    fiducial_vlam::VlocContext &other)
  {
    (void) other;
    LocalizeCameraCvContext cxt{};
    return cxt;
  }

// ==============================================================================
// LocalizeCameraGtsamFactorContext from method
// ==============================================================================

  template<>
  LocalizeCameraGtsamFactorContext LocalizeCameraGtsamFactorContext::from<fiducial_vlam::VlocContext>(
    fiducial_vlam::VlocContext &other)
  {
    LocalizeCameraGtsamFactorContext cxt{other.loc_corner_measurement_sigma_,
                                         other.loc_gtsam_factor_type_,
                                         other.loc_use_marker_covariance_};
    return cxt;
  }

// ==============================================================================
// FiducialMarkerCvContext from method
// ==============================================================================

//  template<>
//  FiducialMarkerCvContext FiducialMarkerCvContext::from<fiducial_vlam::VlocContext>(
//    fiducial_vlam::VlocContext &other)
//  {
//    FiducialMarkerCvContext cxt{other.loc_corner_refinement_method_};
//    cxt.border_color_red_ = 0;
//    cxt.border_color_green_ = 1.0;
//    cxt.border_color_blue_ = 0.;
//    cxt.aruco_dictionary_id_ = other.loc_aruco_dictionary_id_;
//    return cxt;
//  }
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
    VlocDiagnostics diagnostics_;

    VlocContext cxt_{};
    VdetContext det_cxt_{};
    PslContext psl_cxt_{};

    std::unique_ptr<ObservationMakerInterface> observation_maker_{};

    std::unique_ptr<fvlam::LocalizeCameraInterface> localize_camera_{};
    std::unique_ptr<fvlam::FiducialMarkerInterface> fiducial_marker_{};
    fvlam::MarkerMap marker_map_{};

    bool current_loc_camera_algorithm_{};

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

    rclcpp::TimerBase::SharedPtr timer_{};

    void validate_parameters()
    {}

    void validate_psl_parameters()
    {}

    void setup_parameters()
    {
#undef PAMA_PARAM
#define PAMA_PARAM(n, t, d) PAMA_PARAM_INIT(n, t, d)
      PAMA_PARAMS_INIT((*this), cxt_, , VLOC_ALL_PARAMS, validate_parameters)
      PAMA_PARAMS_INIT((*this), det_cxt_, , VDET_ALL_PARAMS, validate_psl_parameters)
      PAMA_PARAMS_INIT((*this), psl_cxt_, , PSL_ALL_PARAMS, validate_psl_parameters)

#undef PAMA_PARAM
#define PAMA_PARAM(n, t, d) PAMA_PARAM_CHANGED(n, t, d)
      PAMA_PARAMS_CHANGED((*this), cxt_, , VLOC_ALL_PARAMS, validate_parameters, RCLCPP_INFO)
      PAMA_PARAMS_CHANGED((*this), det_cxt_, , VDET_ALL_PARAMS, validate_psl_parameters, RCLCPP_INFO)
      PAMA_PARAMS_CHANGED((*this), psl_cxt_, , PSL_ALL_PARAMS, validate_psl_parameters, RCLCPP_INFO)

#undef PAMA_PARAM
#define PAMA_PARAM(n, t, d) PAMA_PARAM_LOG(n, t, d)
      PAMA_PARAMS_LOG((*this), cxt_, , VLOC_ALL_PARAMS, RCLCPP_INFO)
      PAMA_PARAMS_LOG((*this), det_cxt_, , VDET_ALL_PARAMS, RCLCPP_INFO)
      PAMA_PARAMS_LOG((*this), psl_cxt_, , PSL_ALL_PARAMS, RCLCPP_INFO)

#undef PAMA_PARAM
#define PAMA_PARAM(n, t, d) PAMA_PARAM_CHECK_CMDLINE(n, t, d)
      PAMA_PARAMS_CHECK_CMDLINE((*this), , VLOC_ALL_PARAMS VDET_ALL_PARAMS PSL_ALL_PARAMS, RCLCPP_ERROR)
    }

    // Create a LocalizeCameraInterface object if one has not been created yet or
    // the parameter type has changed.
    fvlam::LocalizeCameraInterface &get_lc()
    {
      // Check that a LocalizeCameraInterface has been instantiated
      if (!localize_camera_ || cxt_.loc_camera_algorithm_ != current_loc_camera_algorithm_) {
        if (cxt_.loc_camera_algorithm_ == 1) {
          auto localize_camera_context = fvlam::LocalizeCameraGtsamFactorContext::from(cxt_);
          localize_camera_ = make_localize_camera(localize_camera_context, logger_);

        } else {
          auto localize_camera_context = fvlam::LocalizeCameraCvContext::from(cxt_);
          localize_camera_ = make_localize_camera(localize_camera_context, logger_);
        }

        current_loc_camera_algorithm_ = cxt_.loc_camera_algorithm_;
      }

      return *localize_camera_;
    }

  public:
    explicit VlocNode(const rclcpp::NodeOptions &options) :
      Node("vloc_node", options),
      logger_{*this},
      diagnostics_{now()}
    {
      // Get parameters from the command line
      setup_parameters();

      observation_maker_ = make_observation_maker(
        det_cxt_, *this, logger_,
        [this](const fvlam::CameraInfoMap &camera_info_map,
               const fvlam::ObservationsSynced &observations_synced) -> void
        {
          on_observation_callback(camera_info_map, observations_synced);
        });

#if 0
      // Initialize work objects after parameters have been loaded.
      auto fiducial_marker_context = fvlam::FiducialMarkerCvContext::from(cxt_);
      fiducial_marker_ = make_fiducial_marker(fiducial_marker_context, logger_);

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
          diagnostics_.sub_camera_info_count_ += 1;
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
          diagnostics_.sub_image_raw_count_ += 1;
        });

      sub_map_ = create_subscription<fiducial_vlam_msgs::msg::Map>(
        psl_cxt_.psl_sub_map_topic_,
        1,
        [this](const fiducial_vlam_msgs::msg::Map::UniquePtr msg) -> void
        {
          marker_map_ = fvlam::MarkerMap::from(*msg);
          diagnostics_.sub_map_count_ += 1;
        });
#endif

      // Timer for publishing map info
      timer_ = create_wall_timer(
        std::chrono::milliseconds(1000),
        [this]() -> void
        {
          timer_callback();
        });

      logger_.info()
        << "Using opencv " << CV_VERSION_MAJOR << "."
        << CV_VERSION_MINOR << "." << CV_VERSION_REVISION << "\n"
        << "To calibrate camera - set cal_cmd parameter.\n"
        << "  cal_cmd capture - Capture an image and add it to the set of calibration images.\n"
        << "  cal_cmd calibrate - Take the set of calibration images, do a calibration, and save images, calibration and a report to files.\n"
        << "  cal_cmd status - Report on the number of images in the set of calibration images.\n"
        << "  cal_cmd load_images - Load the set of calibration images from files.\n"
        << "  cal_cmd reset - first time: clear the calibration, second time: clear the set of calibration images.\n"
        << "vloc_node ready";

      (void) sub_camera_info_;
      (void) sub_image_raw_;
      (void) sub_map_;
    }

  private:
    void on_observation_callback(const fvlam::CameraInfoMap &camera_info_map,
                                 const fvlam::ObservationsSynced &observations_synced)
    {
      // Find the camera pose from the observations.
      auto t_map_camera = get_lc().solve_t_map_camera(observations_synced, camera_info_map, marker_map_);

      if (!t_map_camera.is_valid()) {
        diagnostics_.invalid_t_map_camera_count_ += 1;
      }

      if (t_map_camera.is_valid()) {

        // Header to use with pose, odometry, and tf messages
        auto map_frame_id = cxt_.loc_pub_map_frame_id_.empty() ?
                            observations_synced.camera_frame_id() : cxt_.loc_pub_map_frame_id_;
        auto po_header = std_msgs::msg::Header{}
          .set__frame_id(map_frame_id)
          .set__stamp(observations_synced.stamp().to<builtin_interfaces::msg::Time>());

        // Publish the camera pose/odometry in the map frame
        if (cxt_.loc_pub_camera_pose_enable_) {
          auto msg = geometry_msgs::msg::PoseWithCovarianceStamped{}
            .set__header(po_header)
            .set__pose(t_map_camera.to<geometry_msgs::msg::PoseWithCovariance>());
          if (!pub_camera_pose_) {
            pub_camera_pose_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
              cxt_.loc_pub_camera_pose_topic_, 2);
          }
          pub_camera_pose_->publish(msg);
          diagnostics_.pub_camera_pose_count_ += 1;
        }
        if (cxt_.loc_pub_camera_odom_enable_) {
          auto msg = nav_msgs::msg::Odometry{}
            .set__header(po_header)
            .set__child_frame_id(cxt_.loc_pub_camera_odom_child_frame_id_)
            .set__pose(t_map_camera.to<geometry_msgs::msg::PoseWithCovariance>());
          if (!pub_camera_odom_) {
            pub_camera_odom_ = create_publisher<nav_msgs::msg::Odometry>(
              cxt_.loc_pub_camera_odom_topic_, 2);
          }
          pub_camera_odom_->publish(msg);
          diagnostics_.pub_camera_odom_count_ += 1;
        }

        // Find the transform from the base of the robot to the map. Also include the covariance.
        // Note: the covariance values are with respect to the map frame so both t_map_camera and
        // t_map_base have the same covariance.
        auto t_map_base = fvlam::Transform3WithCovariance{
          t_map_camera.tf() * fvlam::Transform3{
            fvlam::Rotate3::RzRyRx(cxt_.loc_t_base_camera_yaw_,
                                   cxt_.loc_t_base_camera_pitch_,
                                   cxt_.loc_t_base_camera_roll_),
            fvlam::Translate3{cxt_.loc_t_base_camera_x_,
                              cxt_.loc_t_base_camera_y_,
                              cxt_.loc_t_base_camera_z_}}.inverse(),
          t_map_camera.cov()};

        // Publish the base pose/odometry in the map frame
        if (cxt_.loc_pub_base_pose_enable_) {
          auto msg = geometry_msgs::msg::PoseWithCovarianceStamped{}
            .set__header(po_header)
            .set__pose(t_map_base.to<geometry_msgs::msg::PoseWithCovariance>());
          if (!pub_base_pose_) {
            pub_base_pose_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
              cxt_.loc_pub_base_pose_topic_, 2);
          }
          pub_base_pose_->publish(msg);
          diagnostics_.pub_base_pose_count_ += 1;
        }
        if (cxt_.loc_pub_base_odom_enable_) {
          auto msg = nav_msgs::msg::Odometry{}
            .set__header(po_header)
            .set__child_frame_id(cxt_.loc_pub_base_odom_child_frame_id_)
            .set__pose(t_map_base.to<geometry_msgs::msg::PoseWithCovariance>());
          if (!pub_base_odom_) {
            pub_base_odom_ = create_publisher<nav_msgs::msg::Odometry>(
              cxt_.loc_pub_base_odom_topic_, 2);
          }
          pub_base_odom_->publish(msg);
          diagnostics_.pub_base_odom_count_ += 1;
        }
#if 0
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
          for (auto &observation : observations) {
            auto t_marker_camera = observation.solve_t_marker_camera(camera_info, marker_map_.marker_length());
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
          for (auto &observation : observations) {
            auto t_camera_marker = observation.solve_t_camera_marker(camera_info, marker_map_.marker_length());
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
          if (!pub_tf_) {
            pub_tf_ = create_publisher<tf2_msgs::msg::TFMessage>(
              "/tf", 2);
          }
          pub_tf_->publish(tfs_msg);
          diagnostics_.pub_tf_count_ += 1;
        }
#endif
      }
    }

#if 0
    void process_image(sensor_msgs::msg::Image::UniquePtr image_msg,
                       sensor_msgs::msg::CameraInfo::UniquePtr camera_info_msg)
    {
      auto stamp = image_msg->header.stamp;
      rclcpp::Time time_stamp{stamp};

      gttic(time_stamp);
      // Convert ROS to OpenCV
      cv_bridge::CvImagePtr gray{cv_bridge::toCvCopy(*image_msg, "mono8")};
      gttoc(time_stamp);

      gttic(grey);
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
      gttoc(grey);

      gttic(color_marked);
      // Detect the markers in this image and create a list of
      // observations.
      auto observations = fiducial_marker_->detect_markers(gray->image, "", fvlam::Stamp{}); // Todo
      gttoc(color_marked);

      if (observations.empty()) {
        diagnostics_.empty_observations_count_ += 1;
      }

      gttic(observations);
      // Annotate the image_marked with the markers that were found.
      if (psl_cxt_.psl_pub_image_marked_enable_) {
        fiducial_marker_->annotate_image_with_detected_markers(color_marked.image, observations);
      }

      // Publish the observations.
      if (psl_cxt_.psl_pub_observations_enable_ &&
          !observations.empty()) {
        auto msg = fiducial_vlam_msgs::msg::Observations{}; // ToDo Fix this
//          .set__header(image_msg->header)
//          .set__camera_info(*camera_info_msg)
//          .set__observations(observations.to<std::vector<fiducial_vlam_msgs::msg::Observation>>());
        if (!pub_observations_) {
          pub_observations_ = create_publisher<fiducial_vlam_msgs::msg::Observations>(
            psl_cxt_.psl_pub_observations_topic_, 2);
        }
        pub_observations_->publish(msg);
        diagnostics_.pub_observations_count_ += 1;
      }

      // Debugging hint: If the markers in color_marked are not outlined
      // in green, then they haven't been detected. If the markers in
      // color_marked are outlined but they have no axes drawn, then
      // no map message has been received. This could be because vmap_node
      // is not running or has not been able to find the starting node.
      // We need a map and observations before we can publish camera
      // localization information.
      if (!marker_map_.empty() && !observations.empty()) {
        auto camera_info = fvlam::CameraInfo::from(*camera_info_msg);

        // Find the camera pose from the observations.
        auto t_map_camera = get_lc().solve_t_map_camera(observations, camera_info, marker_map_);

        if (!t_map_camera.is_valid()) {
          diagnostics_.invalid_t_map_camera_count_ += 1;
        }

        if (t_map_camera.is_valid()) {

          // If annotated images have been requested, then add the annotations now.
          if (psl_cxt_.psl_pub_image_marked_enable_) {
            for (auto &observation : observations) {
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
            if (!pub_camera_pose_) {
              pub_camera_pose_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
                psl_cxt_.psl_pub_camera_pose_topic_, 2);
            }
            pub_camera_pose_->publish(msg);
            diagnostics_.pub_camera_pose_count_ += 1;
          }
          if (psl_cxt_.psl_pub_camera_odom_enable_) {
            auto msg = nav_msgs::msg::Odometry{}
              .set__header(po_header)
              .set__child_frame_id(psl_cxt_.psl_pub_tf_camera_child_frame_id_)
              .set__pose(t_map_camera.to<geometry_msgs::msg::PoseWithCovariance>());
            if (!pub_camera_odom_) {
              pub_camera_odom_ = create_publisher<nav_msgs::msg::Odometry>(
                psl_cxt_.psl_pub_camera_odom_topic_, 2);
            }
            pub_camera_odom_->publish(msg);
            diagnostics_.pub_camera_odom_count_ += 1;
          }

          // Publish the base pose/odometry in the map frame
          if (psl_cxt_.psl_pub_base_pose_enable_) {
            auto msg = geometry_msgs::msg::PoseWithCovarianceStamped{}
              .set__header(po_header)
              .set__pose(t_map_base.to<geometry_msgs::msg::PoseWithCovariance>());
            if (!pub_base_pose_) {
              pub_base_pose_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
                psl_cxt_.psl_pub_base_pose_topic_, 2);
            }
            pub_base_pose_->publish(msg);
            diagnostics_.pub_base_pose_count_ += 1;
          }
          if (psl_cxt_.psl_pub_base_odom_enable_) {
            auto msg = nav_msgs::msg::Odometry{}
              .set__header(po_header)
              .set__child_frame_id(psl_cxt_.psl_pub_tf_base_child_frame_id_)
              .set__pose(t_map_base.to<geometry_msgs::msg::PoseWithCovariance>());
            if (!pub_base_odom_) {
              pub_base_odom_ = create_publisher<nav_msgs::msg::Odometry>(
                psl_cxt_.psl_pub_base_odom_topic_, 2);
            }
            pub_base_odom_->publish(msg);
            diagnostics_.pub_base_odom_count_ += 1;
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
            for (auto &observation : observations) {
              auto t_marker_camera = observation.solve_t_marker_camera(camera_info, marker_map_.marker_length());
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
            for (auto &observation : observations) {
              auto t_camera_marker = observation.solve_t_camera_marker(camera_info, marker_map_.marker_length());
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
            if (!pub_tf_) {
              pub_tf_ = create_publisher<tf2_msgs::msg::TFMessage>(
                "/tf", 2);
            }
            pub_tf_->publish(tfs_msg);
            diagnostics_.pub_tf_count_ += 1;
          }
        }
      }

      // Publish an annotated image if requested. Even if there is no map.
      if (psl_cxt_.psl_pub_image_marked_enable_) {
        // The marking has been happening on the original message.
        // Republish it now.
        if (!pub_image_marked_) {
          pub_image_marked_ = create_publisher<sensor_msgs::msg::Image>(
            psl_cxt_.psl_pub_image_marked_topic_, 2);
        }
        pub_image_marked_->publish(std::move(image_msg));
        diagnostics_.pub_image_marked_count_ += 1;
      }
      gttoc(observations);

      static int ttt = 0;
      if (++ttt > 100) {
        ttt = 0;
#ifdef ENABLE_TIMING
        gtsam::tictoc_print();
#endif
        gtsam::tictoc_reset_();
      }
    }
#endif

    static std::string compose_frame_id(std::string &prefix, uint64_t id)
    {
      std::ostringstream oss;
      oss << prefix
          << std::setfill('0') << std::setw(3)
          << id;
      return oss.str();
    }

    void timer_callback()
    {
      rclcpp::Time time_now{now()};

      // Figure out if there is an map_cmd to process.
      if (!cxt_.loc_cmd_.empty()) {
        std::string cmd{cxt_.loc_cmd_};

        // Reset the cmd_string in preparation for the next command.
        PAMA_SET_PARAM((*this), cxt_, "", loc_cmd, "");

        // Process the new command

        if (cmd == "diagnostics") {
          diagnostics_.report(logger_, now());

        } else {
          logger_.warn() << "Invalid command: " << cmd;
        }
      }
    }
  };

  std::shared_ptr<rclcpp::Node> vloc_node_factory(const rclcpp::NodeOptions &options)
  {
    return std::shared_ptr<rclcpp::Node>(new VlocNode(options));
  }

  void VlocDiagnostics::report(fvlam::Logger &logger, const rclcpp::Time &end_time)
  {
    double per_sec = 1.0 / (end_time - start_time_).seconds();
    logger.info() << "Received camera_info: " << sub_camera_info_count_
                  << " (" << per_sec * sub_camera_info_count_ << " /sec)";
    logger.info() << "Received image_raw: " << sub_image_raw_count_
                  << " (" << per_sec * sub_image_raw_count_ << " /sec)";
    logger.info() << "Received map: " << sub_map_count_
                  << " (" << per_sec * sub_map_count_ << " /sec)";

    logger.info() << "Empty observations: " << empty_observations_count_
                  << " (" << per_sec * empty_observations_count_ << " /sec)";
    logger.info() << "Invalid t_map_camera: " << invalid_t_map_camera_count_
                  << " (" << per_sec * invalid_t_map_camera_count_ << " /sec)";

    logger.info() << "Published observations: " << pub_observations_count_
                  << " (" << per_sec * pub_observations_count_ << " /sec)";
    logger.info() << "Published camera_pose: " << pub_camera_pose_count_
                  << " (" << per_sec * pub_camera_pose_count_ << " /sec)";
    logger.info() << "Published camera_odom: " << pub_camera_odom_count_
                  << " (" << per_sec * pub_camera_odom_count_ << " /sec)";
    logger.info() << "Published base_pose: " << pub_base_pose_count_
                  << " (" << per_sec * pub_base_pose_count_ << " /sec)";
    logger.info() << "Published base_odom: " << pub_base_odom_count_
                  << " (" << per_sec * pub_base_odom_count_ << " /sec)";
    logger.info() << "Published tf: " << pub_tf_count_
                  << " (" << per_sec * pub_tf_count_ << " /sec)";
    logger.info() << "Published imaged_marked: " << pub_image_marked_count_
                  << " (" << per_sec * pub_image_marked_count_ << " /sec)";


    sub_camera_info_count_ = 0;
    sub_image_raw_count_ = 0;
    sub_map_count_ = 0;
    empty_observations_count_ = 0;
    invalid_t_map_camera_count_ = 0;
    pub_observations_count_ = 0;
    pub_camera_pose_count_ = 0;
    pub_camera_odom_count_ = 0;
    pub_base_pose_count_ = 0;
    pub_base_odom_count_ = 0;
    pub_tf_count_ = 0;
    pub_image_marked_count_ = 0;
    start_time_ = end_time;
  }
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(fiducial_vlam::VlocNode)
