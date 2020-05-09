
#include <iomanip>

#include "rclcpp/rclcpp.hpp"

#include "calibrate.hpp"
#include "cv_utils.hpp"
#include "fiducial_math.hpp"
#include "map.hpp"
#include "observation.hpp"
#include "vloc_context.hpp"

#include "cv_bridge/cv_bridge.h"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_msgs/msg/tf_message.hpp"

namespace fiducial_vlam
{

  static void annotate_image_with_marker_axes(
    cv_bridge::CvImage &color_marked,
    const TransformWithCovariance &t_map_camera,
    const std::vector<TransformWithCovariance> &t_map_markers,
    CvFiducialMathInterface &fm,
    const CameraInfoInterface &camera_info)
  {
    // Annotate the image by drawing axes on each marker that was used for the location
    // calculation. This calculation uses the average t_map_camera and the t_map_markers
    // to figure out where the axes should be. This is different from the t_camera_marker
    // that was solved for above.

    // Cache a transform.
    auto tf_t_camera_map = t_map_camera.transform().inverse();

    // Loop through the ids of the markers visible in this image
    for (int i = 0; i < t_map_markers.size(); i += 1) {
      auto &t_map_marker = t_map_markers[i];

      if (t_map_marker.is_valid()) {
        // Calculalte t_camera_marker and draw the axis.
        auto t_camera_marker = TransformWithCovariance(tf_t_camera_map * t_map_marker.transform());
        AnnotateImages::with_marker_axis(color_marked.image, t_camera_marker, camera_info);
      }
    }
  }


// ==============================================================================
// LocalizeCameraProcessImageImpl class
// ==============================================================================

  class LocalizeCameraProcessImageImpl : public ProcessImageInterface
  {
    const VlocContext &cxt_;
    const FiducialMathContext &fm_cxt_;
    CvFiducialMathInterface &fm_;
    std::unique_ptr<LocalizeCameraInterface> cv_lc_{};
    std::unique_ptr<LocalizeCameraInterface> sam_lc_{};

    LocalizeCameraInterface &lc()
    {
      // Select which type of localize camera functionality to use.
      return (cxt_.loc_localize_camera_sam_not_cv_ && sam_lc_) ? *sam_lc_ : *cv_lc_;
    }


  public:
    explicit LocalizeCameraProcessImageImpl(const VlocContext &cxt,
                                            const FiducialMathContext &fm_cxt,
                                            CvFiducialMathInterface &fm) :
      cxt_{cxt}, fm_cxt_{fm_cxt}, fm_{fm},
      cv_lc_{make_cv_localize_camera(fm_cxt_)},
      sam_lc_{make_sam_localize_camera(fm_cxt_, *cv_lc_)}
    {}

    Observations process_image(std::shared_ptr<cv_bridge::CvImage> &gray,
                               const rclcpp::Time &time_stamp,
                               cv::Mat &color_marked) override
    {
      // Detect the markers in this image and create a list of
      // observations.
      return fm_.detect_markers(*gray, color_marked);
    }

    TransformWithCovariance solve_t_map_camera(const Observations &observations,
                                               const CameraInfoInterface &camera_info,
                                               const Map &map) override
    {
      return lc().solve_t_map_camera(observations, camera_info, map);
    }
  };

// ==============================================================================
// VlocNode class
// ==============================================================================

  class VlocNode : public rclcpp::Node
  {
    rclcpp::Logger ros_logger_inst_;
    VlocContext cxt_{};
    FiducialMathContext fm_cxt_{};
    CalibrateContext cal_cxt_{};
    std::unique_ptr<CvFiducialMathInterface> fm_;
    std::unique_ptr<ProcessImageInterface> lc_pi_{};
    std::unique_ptr<CalibrateCameraInterface> cc_pi_{};

    std::unique_ptr<Map> map_{};
    std::unique_ptr<sensor_msgs::msg::CameraInfo> camera_info_msg_{};
    std_msgs::msg::Header::_stamp_type last_image_stamp_{};

    rclcpp::Publisher<fiducial_vlam_msgs::msg::Observations>::SharedPtr observations_pub_{};
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr camera_pose_pub_{};
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr base_pose_pub_{};
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_message_pub_{};
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr camera_odometry_pub_{};
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr base_odometry_pub_{};
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_marked_pub_{};

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_raw_sub_;
    rclcpp::Subscription<fiducial_vlam_msgs::msg::Map>::SharedPtr map_sub_;
    rclcpp::TimerBase::SharedPtr calibrate_timer_{};

    void validate_parameters()
    {
      cxt_.t_camera_base_ = TransformWithCovariance(TransformWithCovariance::mu_type{
        cxt_.t_camera_base_x_, cxt_.t_camera_base_y_, cxt_.t_camera_base_z_,
        cxt_.t_camera_base_roll_, cxt_.t_camera_base_pitch_, cxt_.t_camera_base_yaw_});
    }

    void validate_fm_parameters()
    {}

    void validate_cal_parameters()
    {}

    void setup_parameters()
    {
      // Do the vloc_node parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER((*this), cxt_, n, t, d)
      CXT_MACRO_INIT_PARAMETERS(VLOC_ALL_PARAMS, validate_parameters)

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(cxt_, n, t)
      CXT_MACRO_REGISTER_PARAMETERS_CHANGED((*this), VLOC_ALL_PARAMS, validate_parameters)

      // Do the fiducial_math parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER((*this), fm_cxt_, n, t, d)
      CXT_MACRO_INIT_PARAMETERS(FM_ALL_PARAMS, validate_fm_parameters)

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(fm_cxt_, n, t)
      CXT_MACRO_REGISTER_PARAMETERS_CHANGED((*this), FM_ALL_PARAMS, validate_fm_parameters)

      // Do the calibrate  parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER((*this), cal_cxt_, n, t, d)
      CXT_MACRO_INIT_PARAMETERS(CAL_ALL_PARAMS, validate_cal_parameters)

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(cal_cxt_, n, t)
      CXT_MACRO_REGISTER_PARAMETERS_CHANGED((*this), CAL_ALL_PARAMS, validate_cal_parameters)

      // Display all the parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_SORTED_PARAMETER(cxt_, n, t, d)
      CXT_MACRO_LOG_SORTED_PARAMETERS(RCLCPP_INFO, get_logger(), "VlocNode Parameters", VLOC_ALL_PARAMS)

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_SORTED_PARAMETER(fm_cxt_, n, t, d)
      CXT_MACRO_LOG_SORTED_PARAMETERS(RCLCPP_INFO, get_logger(), "FiducialMath Parameters", FM_ALL_PARAMS)

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_SORTED_PARAMETER(cal_cxt_, n, t, d)
      CXT_MACRO_LOG_SORTED_PARAMETERS(RCLCPP_INFO, get_logger(), "Calibrate Parameters", CAL_ALL_PARAMS)

      // Check that all command line parameters are registered
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_CHECK_CMDLINE_PARAMETER(n, t, d)
      CXT_MACRO_CHECK_CMDLINE_PARAMETERS((*this), VLOC_ALL_PARAMS FM_ALL_PARAMS CAL_ALL_PARAMS)
    }

    ProcessImageInterface &pi()
    {
      return cxt_.loc_calibrate_not_loocalize_ ? *cc_pi_ : *lc_pi_;
    }

    bool publish_captured_image_marked()
    {
      return cxt_.loc_calibrate_not_loocalize_ &&
             cc_pi_->calibration_complete();
    }

  public:
    VlocNode(const rclcpp::NodeOptions &options) :
      Node("vloc_node", options),
      ros_logger_inst_{get_logger()},
      fm_{make_cv_fiducial_math(fm_cxt_)},
      lc_pi_{std::make_unique<LocalizeCameraProcessImageImpl>(cxt_, fm_cxt_, *fm_)},
      cc_pi_{make_calibrate_camera(ros_logger_inst_, cal_cxt_)}
    {
      // Get parameters from the command line
      setup_parameters();

      // ROS publishers. Initialize after parameters have been loaded.
      observations_pub_ = create_publisher<fiducial_vlam_msgs::msg::Observations>(
        cxt_.fiducial_observations_pub_topic_, 16);

      if (cxt_.publish_camera_pose_) {
        camera_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          cxt_.camera_pose_pub_topic_, 16);
      }
      if (cxt_.publish_base_pose_) {
        base_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          cxt_.base_pose_pub_topic_, 16);
      }
      if (cxt_.publish_tfs_) {
        tf_message_pub_ = create_publisher<tf2_msgs::msg::TFMessage>(
          "/tf", 16);
      }
      if (cxt_.publish_camera_odom_) {
        camera_odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(
          cxt_.camera_odometry_pub_topic_, 16);
      }
      if (cxt_.publish_base_odom_) {
        base_odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(
          cxt_.base_odometry_pub_topic_, 16);
      }
      if (cxt_.publish_image_marked_) {
        image_marked_pub_ = create_publisher<sensor_msgs::msg::Image>(
          cxt_.image_marked_pub_topic_, 16);
      }

      // ROS subscriptions
      auto camera_info_qos = cxt_.sub_camera_info_best_effort_not_reliable_ ?
                             rclcpp::QoS{rclcpp::SensorDataQoS()} :
                             rclcpp::QoS{rclcpp::ServicesQoS()};
      camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        cxt_.camera_info_sub_topic_,
        camera_info_qos,
        [this](sensor_msgs::msg::CameraInfo::UniquePtr msg) -> void
        {
          // Save this message because it will be used for the next image message.
          // Note: we are taking ownership of this object
          camera_info_msg_ = std::move(msg);
        });

      image_raw_sub_ = create_subscription<sensor_msgs::msg::Image>(
        cxt_.image_raw_sub_topic_,
        rclcpp::ServicesQoS(rclcpp::KeepLast(1)),
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
            RCLCPP_DEBUG(get_logger(), "Ignore image message because no camera_info has been received yet.");

          } else if ((stamp.nanosec == 0l && stamp.sec == 0l) || stamp == last_image_stamp_) {
            RCLCPP_DEBUG(get_logger(), "Ignore image message because stamp is zero or the same as the previous.");

            // If we have just done a calibration and want to publish the marked captured
            // images then there is nothing to do with this image so ignore it.
          } else if (!publish_captured_image_marked()) {

            // rviz doesn't like it when time goes backward when a bag is played again.
            // The stamp_msgs_with_current_time_ parameter can help this by replacing the
            // image message time with the current time.
            stamp = cxt_.stamp_msgs_with_current_time_ ? builtin_interfaces::msg::Time(now()) : stamp;
            process_image(std::move(msg), std::move(camera_info_msg_), stamp);
          }

          last_image_stamp_ = stamp;
        });

      map_sub_ = create_subscription<fiducial_vlam_msgs::msg::Map>(
        cxt_.fiducial_map_sub_topic_,
        16,
        [this](const fiducial_vlam_msgs::msg::Map::UniquePtr msg) -> void
        {
          map_ = std::make_unique<Map>(*msg);
        });

      // Timer for publishing map info
      calibrate_timer_ = create_wall_timer(
        std::chrono::milliseconds(250),
        [this]() -> void
        {
          calibrate_timer_callback();
        });

      RCLCPP_INFO(get_logger(), "Using opencv %d.%d.%d", CV_VERSION_MAJOR, CV_VERSION_MINOR, CV_VERSION_REVISION);
      RCLCPP_INFO(get_logger(), "vloc_node ready");

      (void) camera_info_sub_;
      (void) image_raw_sub_;
      (void) map_sub_;
      (void) calibrate_timer_;
    }

  private:
    void process_image(sensor_msgs::msg::Image::UniquePtr image_msg,
                       std::unique_ptr<sensor_msgs::msg::CameraInfo> camera_info_msg,
                       const std_msgs::msg::Header::_stamp_type &stamp)
    {
      rclcpp::Time time_stamp{stamp};

      // Convert ROS to OpenCV
      cv_bridge::CvImagePtr gray{cv_bridge::toCvCopy(*image_msg, "mono8")};

      // If we are going to publish an annotated image, make a copy of
      // the original message image. If no annotated image is to be published,
      // then just make an empty image pointer. The routines need to check
      // that the pointer is valid before drawing into it.
      cv_bridge::CvImage color_marked;

//      if (cxt_.publish_image_marked_ &&
//          count_subscribers(cxt_.image_marked_pub_topic_) > 0) {
      if (cxt_.publish_image_marked_) {

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
      }

      // Detect the markers in this image and create a list of
      // observations.
      auto observations = pi().process_image(gray, time_stamp, color_marked.image);

      // Publish the observations.
      if (observations.size()) {
        auto observations_msg = observations.to_msg(stamp, image_msg->header.frame_id, *camera_info_msg);
        observations_pub_->publish(observations_msg);
      }

      // Debugging hint: If the markers in color_marked are not outlined
      // in green, then they haven't been detected. If the markers in
      // color_marked are outlined but they have no axes drawn, then
      // no map message has been received. This could be because vmap_node
      // is not running or has not been able to find the starting node.
      // We need a map and observations before we can publish camera
      // localization information.
      if (map_ && !observations.observations().empty()) {
        auto camera_info{make_camera_info(*camera_info_msg)};

        // Find the camera pose from the observations.
        auto t_map_camera{pi().solve_t_map_camera(observations, *camera_info, *map_)};

        if (t_map_camera.is_valid()) {

          // If annotated images have been requested, then add the annotations now.
          if (color_marked.header.stamp != std_msgs::msg::Header::_stamp_type{}) {
            auto t_map_markers = map_->find_t_map_markers(observations);
            annotate_image_with_marker_axes(color_marked, t_map_camera, t_map_markers, *fm_, *camera_info);
          }

          // Find the transform from the base of the robot to the map. Also include the covariance.
          // Note: the covariance values are with respect to the map frame so both t_map_camera and
          // t_map_base have the same covariance.
          TransformWithCovariance t_map_base{
            t_map_camera.transform() * cxt_.t_camera_base_.transform(),
            t_map_camera.cov()};

          // Publish the camera an/or base pose in the map frame
          if (cxt_.publish_camera_pose_) {
            auto pose_msg = to_PoseWithCovarianceStamped_msg(t_map_camera, stamp, cxt_.map_frame_id_);
            // add some fixed variance for now.
            add_fixed_covariance(pose_msg.pose);
            camera_pose_pub_->publish(pose_msg);
          }
          if (cxt_.publish_base_pose_) {
            auto pose_msg = to_PoseWithCovarianceStamped_msg(t_map_base, stamp, cxt_.map_frame_id_);
            // add some fixed variance for now.
            add_fixed_covariance(pose_msg.pose);
            base_pose_pub_->publish(pose_msg);
          }

          // Publish odometry of the camera and/or the base.
          if (cxt_.publish_camera_odom_) {
            auto odom_msg = to_odom_message(stamp, cxt_.camera_frame_id_, t_map_camera);
            add_fixed_covariance(odom_msg.pose);
            camera_odometry_pub_->publish(odom_msg);
          }
          if (cxt_.publish_base_odom_) {
            auto odom_msg = to_odom_message(stamp, cxt_.base_frame_id_, t_map_base);
            add_fixed_covariance(odom_msg.pose);
            base_odometry_pub_->publish(odom_msg);
          }

          // Also publish the camera's tf
          if (cxt_.publish_tfs_) {
            auto tf_message = to_tf_message(stamp, t_map_camera, t_map_base);
            tf_message_pub_->publish(tf_message);
          }

          // if requested, publish the camera tf as determined from each marker.
          if (cxt_.publish_tfs_per_marker_) {
            auto t_map_cameras = markers_t_map_cameras(observations, *camera_info, *map_);
            auto tf_message = to_markers_tf_message(stamp, observations, t_map_cameras);
            if (!tf_message.transforms.empty()) {
              tf_message_pub_->publish(tf_message);
            }
          }
        }
      }

      // Publish an annotated image if requested. Even if there is no map.
      if (color_marked.image.dims != 0) {
        // The marking has been happening on the original message.
        // Republish it now.
        image_marked_pub_->publish(std::move(image_msg));
      }
    }

    nav_msgs::msg::Odometry to_odom_message(std_msgs::msg::Header::_stamp_type stamp,
                                            const std::string &child_frame_id,
                                            const TransformWithCovariance &t)
    {
      nav_msgs::msg::Odometry odom_message;

      odom_message.header.stamp = stamp;
      odom_message.header.frame_id = cxt_.map_frame_id_;
      odom_message.child_frame_id = child_frame_id;
      odom_message.pose = to_PoseWithCovariance_msg(t);
      return odom_message;
    }

    tf2_msgs::msg::TFMessage to_tf_message(std_msgs::msg::Header::_stamp_type stamp,
                                           const TransformWithCovariance &t_map_camera,
                                           const TransformWithCovariance &t_map_base)
    {
      tf2_msgs::msg::TFMessage tf_message;

      geometry_msgs::msg::TransformStamped msg;
      msg.header.stamp = stamp;
      msg.header.frame_id = cxt_.map_frame_id_;

      // The camera_frame_id parameter is non-empty to publish the camera tf.
      // The base_frame_id parameter is non-empty to publish the base tf.
      if (!cxt_.camera_frame_id_.empty()) {
        msg.child_frame_id = cxt_.camera_frame_id_;
        msg.transform = tf2::toMsg(t_map_camera.transform());
        tf_message.transforms.emplace_back(msg);
      }
      if (!cxt_.base_frame_id_.empty()) {
        msg.child_frame_id = cxt_.base_frame_id_;
        msg.transform = tf2::toMsg(t_map_base.transform());
        tf_message.transforms.emplace_back(msg);
      }

      return tf_message;
    }

    tf2_msgs::msg::TFMessage to_markers_tf_message(
      std_msgs::msg::Header::_stamp_type stamp,
      const Observations &observations,
      const std::vector<TransformWithCovariance> &t_map_cameras)
    {
      tf2_msgs::msg::TFMessage tf_message;

      geometry_msgs::msg::TransformStamped msg;
      msg.header.stamp = stamp;
      msg.header.frame_id = cxt_.map_frame_id_;

      for (int i = 0; i < observations.size(); i += 1) {
        auto &observation = observations.observations()[i];
        auto &t_map_camera = t_map_cameras[i];

        if (t_map_camera.is_valid()) {

          if (!cxt_.camera_frame_id_.empty()) {
            std::ostringstream oss_child_frame_id;
            oss_child_frame_id << cxt_.camera_frame_id_ << "_m" << std::setfill('0') << std::setw(3)
                               << observation.id();
            msg.child_frame_id = oss_child_frame_id.str();
            msg.transform = tf2::toMsg(t_map_camera.transform());
            tf_message.transforms.emplace_back(msg);
          }
        }
      }

      return tf_message;
    }

    std::vector<TransformWithCovariance> markers_t_map_cameras(
      const Observations &observations,
      const CameraInfoInterface &camera_info,
      const Map &map)
    {
      std::vector<TransformWithCovariance> t_map_cameras;

      for (const auto &observation : observations.observations()) {
        Observations single_observation{};
        single_observation.add(observation);
        auto t_map_camera = pi().solve_t_map_camera(single_observation, camera_info, map);
        if (t_map_camera.is_valid()) {
          t_map_cameras.emplace_back(t_map_camera);
        }
      }

      return t_map_cameras;
    }

    void add_fixed_covariance(geometry_msgs::msg::PoseWithCovariance &pwc)
    {
      return; // don't change covariance.
      // A hack for now.
      // Seeing how rviz2 interprets these values, allows me to confirm which columns represent
      // which variables.
      pwc.covariance[0] = 96e-3; // along fixed x axis
      pwc.covariance[7] = 24e-3; // along fixed y axis
      pwc.covariance[14] = 6e-3; // along fixed z axis
      pwc.covariance[21] = 36e-3; // Not quite sure how rotation works. ??
      pwc.covariance[28] = 12e-3; //
      pwc.covariance[35] = 4e-3; //
    }


    void calibrate_timer_callback()
    {
      rclcpp::Time time_now{now()};

      // Figure out if there is a command to process.
      if (!cal_cxt_.cal_cmd_.empty()) {
        std::string cmd{cal_cxt_.cal_cmd_};

        // Reset the cmd_string in preparation for the next command.
        CXT_MACRO_SET_PARAMETER((*this), cal_cxt_, cal_cmd, "");

        // If we are not in calibrate mode, then don't send the command.
        if (!cxt_.loc_calibrate_not_loocalize_) {
          RCLCPP_ERROR(get_logger(), "Cannot execute cal_cmd when not in calibrate mode");

        } else {
          auto ret_str = cc_pi_->cal_cmd(cmd, time_now);
          if (!ret_str.empty()) {
            RCLCPP_INFO(get_logger(), "cal_cmd '%s' response:\n%s", cmd.c_str(), ret_str.c_str());
          }
        }
      }

      // Give the camera calibrator process some background time
      if (cxt_.loc_calibrate_not_loocalize_) {
        auto ret_str = cc_pi_->on_timer(time_now);
        if (!ret_str.empty()) {
          RCLCPP_INFO(get_logger(), "cal_on_timer response:\n%s", ret_str.c_str());
        }
      }

      // If we have just done a calibration and want to publish the marked captured
      // images then do it.
      if (publish_captured_image_marked()) {
        cv::Mat captured_image_marked;
        cc_pi_->get_captured_image_marked(time_now, captured_image_marked);
        if (captured_image_marked.dims != 0) {

          // build up an image message and publish it.
          std_msgs::msg::Header header;
          header.stamp = time_now;
          header.frame_id = "captured_image_marked";
          cv_bridge::CvImage cv_image{header, "bgr8", captured_image_marked};
          image_marked_pub_->publish(*cv_image.toImageMsg());
        }
      }
    }

  };

  std::shared_ptr<rclcpp::Node> vloc_node_factory(const rclcpp::NodeOptions &options)
  {
    return std::shared_ptr<rclcpp::Node>(new VlocNode(options));
  }
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(fiducial_vlam::VlocNode)
