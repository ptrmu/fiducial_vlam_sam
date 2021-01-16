
#include <iomanip>
#include <iostream>
#include <fstream>

#include "cv_bridge/cv_bridge.h"
#include "fvlam/camera_info.hpp"
#include "fvlam/localize_camera_interface.hpp"
#include "fvlam/logger.hpp"
#include "fvlam/marker.hpp"
#include "fvlam/observation.hpp"
#include "fvlam/transform3_with_covariance.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "calibrate.hpp"
#include "cv_utils.hpp"
#include "fiducial_math.hpp"
#include "logger_ros2.hpp"
#include "map.hpp"
#include "observation.hpp"
#include "vloc_context.hpp"


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
// annotate_image_with_marker_axes routine
// ==============================================================================

//  static void annotate_image_with_marker_axes(
//    cv_bridge::CvImage &color_marked,
//    const TransformWithCovariance &t_map_camera,
//    const std::vector<TransformWithCovariance> &t_map_markers,
//    CvFiducialMathInterface &fm,
//    const CameraInfoInterface &camera_info)
//  {
//    (void) fm;
//
//    // Annotate the image by drawing axes on each marker that was used for the location
//    // calculation. This calculation uses the average t_map_camera and the t_map_markers
//    // to figure out where the axes should be. This is different from the t_camera_marker
//    // that was solved for above.
//
//    // Cache a transform.
//    auto tf_t_camera_map = t_map_camera.transform().inverse();
//
//    // Loop through the ids of the markers visible in this image
//    for (size_t i = 0; i < t_map_markers.size(); i += 1) {
//      auto &t_map_marker = t_map_markers[i];
//
//      if (t_map_marker.is_valid()) {
//        // Calculalte t_camera_marker and draw the axis.
//        auto t_camera_marker = TransformWithCovariance(tf_t_camera_map * t_map_marker.transform());
//        AnnotateImages::with_marker_axis(color_marked.image, t_camera_marker, camera_info);
//      }
//    }
//  }

// ==============================================================================
// LocalizeCameraProcessImageImpl class
// ==============================================================================

  class LocalizeCameraProcessImageImpl : public ProcessImageInterface
  {
    const VlocContext &cxt_;
    CvFiducialMathInterface &fm_;
    std::unique_ptr<LocalizeCameraInterface> cv_lc_{};
    std::unique_ptr<LocalizeCameraInterface> sam_lc_{};

    LocalizeCameraInterface &lc()
    {
      // Select which type of localize camera functionality to use.
      return (cxt_.loc_camera_sam_not_cv_ && sam_lc_) ? *sam_lc_ : *cv_lc_;
    }


  public:
    explicit LocalizeCameraProcessImageImpl(const VlocContext &cxt,
                                            CvFiducialMathInterface &fm) :
      cxt_{cxt}, fm_{fm},
      cv_lc_{make_cv_localize_camera(cxt_)},
      sam_lc_{make_sam_localize_camera(cxt_, *cv_lc_)}
    {}

    Observations process_image(std::shared_ptr<cv_bridge::CvImage> &gray,
                               const rclcpp::Time &time_stamp,
                               cv::Mat &color_marked) override
    {
      // Detect the markers in this image and create a list of
      // observations.
      return fm_.detect_markers(*gray, time_stamp, color_marked);
    }

    TransformWithCovariance solve_t_map_camera(const Observations &observations,
                                               const CameraInfoInterface &camera_info,
                                               const Map &map) override
    {
      return lc().solve_t_map_camera(observations, camera_info, map);
    }
  };


  std::unique_ptr<ProcessImageInterface> make_localize_camera_process_image(const VlocContext &cxt,
                                                                            CvFiducialMathInterface &fm)
  {
    return std::make_unique<LocalizeCameraProcessImageImpl>(cxt, fm);
  }


//  static void save_observations(const rclcpp::Time &time_stamp,
//                                const Observations &observations,
//                                const sensor_msgs::msg::CameraInfo &camera_info_msg)
//  {
//    double marker_length = 0.21;
//    auto k = camera_info_msg.k;
//    auto d = camera_info_msg.d;
//    auto camera_info{make_camera_info(camera_info_msg)};
//    std::stringstream ss{};
//    ss.precision(12);
//    ss << std::scientific
//       << "{ \"stamp\": " << int((time_stamp.seconds() - 1606886000.) * 1000) << ",\n"
//       << "\"camera_info\": {"
//       << "\"height\": " << int(camera_info_msg.height) << ","
//       << "\"width\": " << int(camera_info_msg.width) << ",\n"
//       << "  \"K\": [" << k[0] << "," << k[1] << "," << k[2] << "," << k[3] << "," << k[4]
//       << "," << k[5] << "," << k[6] << "," << k[7] << "," << k[8] << "],\n"
//       << "  \"D\": [" << d[0] << "," << d[1] << "," << d[2] << "," << d[3] << "," << d[4] << "]},\n"
//       << "\"observations\": [\n";
//
//    bool first_record = true;
//    for (auto &observation : observations.observations()) {
//      auto t_camera_marker = CvUtils::solve_t_camera_marker(observation, *camera_info, marker_length);
//      auto mu = t_camera_marker.mu();
//      ss << (first_record ? "  {" : ", {")
//         << "\"id\": " << observation.id() << ",\n"
//         << "  \"corners_f_image\": ["
//         << "[" << observation.x0() << "," << observation.y0() << "],"
//         << "[" << observation.x1() << "," << observation.y1() << "],"
//         << "[" << observation.x2() << "," << observation.y2() << "],"
//         << "[" << observation.x3() << "," << observation.y3() << "]"
//         << "],\n"
//         << "  \"marker_f_camera\": [" << mu[0] << "," << mu[1] << ","
//         << mu[2] << "," << mu[3] << "," << mu[4] << "," << mu[5] << "]"
//         << "}\n";
//      first_record = false;
//    }
//
//    ss << "]}";
//
////    std::cout << ss.str() << std::endl;
//
////    cv::FileStorage fs("obs", cv::FileStorage::WRITE | cv::FileStorage::MEMORY | cv::FileStorage::FORMAT_JSON);
////
////    fs << "stamp" << int((time_stamp.seconds() - 1606886000.) * 1000)
////       << "camera_info" << "{:"
////       << "height" << int(camera_info_msg.height)
////       << "width" << int(camera_info_msg.width)
////       << "K" << "[" << k[0] << k[1] << k[2] << k[3] << k[4] << k[5] << k[6] << k[7] << k[8] << "]"
////       << "D" << "[" << d[0] << d[1] << d[2] << d[3] << d[4] << "]"
////       << "}"
////       << "observations" << "[";
////
////    for (auto &observation : observations.observations()) {
////      auto t_camera_marker = CvUtils::solve_t_camera_marker(observation, *camera_info, marker_length);
////      auto mu = t_camera_marker.mu();
////      fs << "{:"
////         << "id" << observation.id()
////         << "corners_f_image" << "["
////         << "[" << observation.x0() << observation.y0() << "]"
////         << "[" << observation.x1() << observation.y1() << "]"
////         << "[" << observation.x2() << observation.y2() << "]"
////         << "[" << observation.x3() << observation.y3() << "]"
////         << "]"
////         << "marker_f_camera" << "[" << mu[0] << mu[1] << mu[2] << mu[3] << mu[4] << mu[5] << "]"
////         << "}";
////    }
////
////    fs << "]";
////    auto s = fs.releaseAndGetString();
////    std::cout << s << std::endl;
//
//    static bool inited = false;
//    std::ofstream os("observations.json", std::ios::out | (inited ? std::ios::app : std::ios::trunc));
//    if (!inited) {
//      os << "{\"measurements\": [\n";
//    } else {
//      os << ",\n";
//    }
//    os << ss.str();
//    inited = true;
//
//    // Add "]}" to end of file to properly terminate the file.
//  }

// ==============================================================================
// VlocNode class
// ==============================================================================

  class VlocNode : public rclcpp::Node
  {
    rclcpp::Logger ros_logger_inst_;
    LoggerRos2 logger_;
    VlocContext cxt_{};
    PslContext psl_cxt_{};
    std::unique_ptr<fvlam::LocalizeCameraInterface> localize_camera_{};
    std::unique_ptr<fvlam::FiducialMarkerInterface> fiducial_marker_{};
    fvlam::MarkerMap marker_map_{1.0};

    CalibrateContext cal_cxt_{};
    std::unique_ptr<SmoothObservationsInterface> so_{};
    std::unique_ptr<CvFiducialMathInterface> fm_{};
    std::unique_ptr<ProcessImageInterface> lc_pi_{};
    std::unique_ptr<CalibrateCameraInterface> cc_pi_{};

    std::unique_ptr<Map> map_{};
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

    rclcpp::TimerBase::SharedPtr calibrate_timer_{};

    void validate_parameters()
    {
      cxt_.loc_t_camera_base_ = TransformWithCovariance(TransformWithCovariance::mu_type{
        cxt_.loc_t_camera_base_x_, cxt_.loc_t_camera_base_y_, cxt_.loc_t_camera_base_z_,
        cxt_.loc_t_camera_base_roll_, cxt_.loc_t_camera_base_pitch_, cxt_.loc_t_camera_base_yaw_});
    }

    void validate_psl_parameters()
    {}

    void validate_cal_parameters()
    {}

    void setup_parameters()
    {
#undef PAMA_PARAM
#define PAMA_PARAM(n, t, d) PAMA_PARAM_INIT(n, t, d)
      PAMA_PARAMS_INIT((*this), cxt_, "", VLOC_ALL_PARAMS, validate_parameters)
      PAMA_PARAMS_INIT((*this), psl_cxt_, "", PSL_ALL_PARAMS, validate_psl_parameters)
      PAMA_PARAMS_INIT((*this), cal_cxt_, "", CAL_ALL_PARAMS, validate_cal_parameters)

#undef PAMA_PARAM
#define PAMA_PARAM(n, t, d) PAMA_PARAM_CHANGED(n, t, d)
      PAMA_PARAMS_CHANGED((*this), cxt_, "", VLOC_ALL_PARAMS, validate_parameters, RCLCPP_INFO)
      PAMA_PARAMS_CHANGED((*this), psl_cxt_, "", PSL_ALL_PARAMS, validate_psl_parameters, RCLCPP_INFO)
      PAMA_PARAMS_CHANGED((*this), cal_cxt_, "", CAL_ALL_PARAMS, validate_cal_parameters, RCLCPP_INFO)

#undef PAMA_PARAM
#define PAMA_PARAM(n, t, d) PAMA_PARAM_LOG(n, t, d)
      PAMA_PARAMS_LOG((*this), cxt_, "", VLOC_ALL_PARAMS, RCLCPP_INFO)
      PAMA_PARAMS_LOG((*this), psl_cxt_, "", PSL_ALL_PARAMS, RCLCPP_INFO)
      PAMA_PARAMS_LOG((*this), cal_cxt_, "", CAL_ALL_PARAMS, RCLCPP_INFO)

#undef PAMA_PARAM
#define PAMA_PARAM(n, t, d) PAMA_PARAM_CHECK_CMDLINE(n, t, d)
      PAMA_PARAMS_CHECK_CMDLINE((*this), "", VLOC_ALL_PARAMS PSL_ALL_PARAMS CAL_ALL_PARAMS, RCLCPP_ERROR)
    }

    ProcessImageInterface &pi()
    {
      return *lc_pi_;
    }

    std::unique_ptr<fvlam::LocalizeCameraInterface>
  public:
    VlocNode(const rclcpp::NodeOptions &options) :
      Node("vloc_node", options),
      ros_logger_inst_{get_logger()},
      logger_{*this}
    {
      // Get parameters from the command line
      setup_parameters();

      // Initialize work objects after parameters have been loaded.
      auto fiducial_marker_context = fvlam::FiducialMarkerCvContext();
      fiducial_marker_ = make_fiducial_marker(fiducial_marker_context, logger_);

      auto localize_camera_context = fvlam::LocalizeCameraCvContext();
      localize_camera_ = make_localize_camera(localize_camera_context, logger_);

      fm_ = make_cv_fiducial_math(cxt_, *so_);
      lc_pi_ = make_localize_camera_process_image(cxt_, *fm_);

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
            RCLCPP_DEBUG(get_logger(), "Ignore image message because no camera_info has been received yet.");

          } else if ((stamp.nanosec == 0l && stamp.sec == 0l) || stamp == last_image_stamp_) {
            RCLCPP_DEBUG(get_logger(), "Ignore image message because stamp is zero or the same as the previous.");

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
      (void) calibrate_timer_;
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
      }

      // Detect the markers in this image and create a list of
      // observations.
      auto observations = fiducial_marker_->detect_markers(gray->image);

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
      if (map_ && !observations.observations().empty()) {
        auto camera_info = fvlam::CameraInfo::from(*camera_info_msg);

        // Find the camera pose from the observations.
        auto t_map_camera = localize_camera_->solve_t_map_camera(observations, camera_info, marker_map_);

        if (t_map_camera.is_valid()) {

          // If annotated images have been requested, then add the annotations now.
          if (color_marked.header.stamp != std_msgs::msg::Header::_stamp_type{}) {
            fiducial_marker_->annotate_image_with_detected_markers(color_marked.image, observations);
            for (auto &observation : observations.observations()) {
              auto marker = marker_map_.find_marker(observation.id());
              if (marker != nullptr) {
                auto t_camera_marker = t_map_camera.tf().inverse() * marker->t_world_marker().tf();
                fiducial_marker_->annotate_image_with_marker_axis(color_marked.image, t_camera_marker, camera_info);
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
            add_fixed_covariance(msg.pose);
            pub_camera_pose_->publish(msg);
          }
          if (psl_cxt_.psl_pub_camera_odom_enable_) {
            auto msg = nav_msgs::msg::Odometry{}
              .set__header(po_header)
              .set__child_frame_id(psl_cxt_.psl_pub_tf_camera_child_frame_id_)
              .set__pose(t_map_camera.to<geometry_msgs::msg::PoseWithCovariance>());
            add_fixed_covariance(msg.pose);
            pub_camera_odom_->publish(msg);
          }

          // Publish the base pose/odometry in the map frame
          if (psl_cxt_.psl_pub_base_pose_enable_) {
            auto msg = geometry_msgs::msg::PoseWithCovarianceStamped{}
              .set__header(po_header)
              .set__pose(t_map_base.to<geometry_msgs::msg::PoseWithCovariance>());
            add_fixed_covariance(msg.pose);
            pub_camera_pose_->publish(msg);
          }
          if (psl_cxt_.psl_pub_base_odom_enable_) {
            auto msg = nav_msgs::msg::Odometry{}
              .set__header(po_header)
              .set__child_frame_id(psl_cxt_.psl_pub_tf_base_child_frame_id_)
              .set__pose(t_map_base.to<geometry_msgs::msg::PoseWithCovariance>());
            add_fixed_covariance(msg.pose);
            pub_camera_odom_->publish(msg);
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
              auto marker = marker_map_.find_marker(observation.id());
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
      if (color_marked.image.dims != 0) {
        // The marking has been happening on the original message.
        // Republish it now.
        pub_image_marked_->publish(std::move(image_msg));
      }
    }

    std::string compose_frame_id(std::string &prefix, uint64_t id)
    {
      std::ostringstream oss;
      oss << prefix
          << std::setfill('0') << std::setw(3)
          << id;
      return oss.str();
    }

    nav_msgs::msg::Odometry to_odom_message(std_msgs::msg::Header::_stamp_type stamp,
                                            const std::string &child_frame_id,
                                            const TransformWithCovariance &t)
    {
      nav_msgs::msg::Odometry odom_message;

      odom_message.header.stamp = stamp;
      odom_message.header.frame_id = psl_cxt_.psl_pub_map_frame_id_;
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
      msg.header.frame_id = psl_cxt_.psl_pub_map_frame_id_;

      // The psl_camera_frame_id parameter is non-empty to publish the camera tf.
      // The psl_base_frame_id parameter is non-empty to publish the base tf.
      if (!psl_cxt_.psl_pub_tf_camera_child_frame_id_.empty()) {
        msg.child_frame_id = psl_cxt_.psl_pub_tf_camera_child_frame_id_;
        msg.transform = tf2::toMsg(t_map_camera.transform());
        tf_message.transforms.emplace_back(msg);
      }
      if (!psl_cxt_.psl_pub_tf_base_child_frame_id_.empty()) {
        msg.child_frame_id = psl_cxt_.psl_pub_tf_base_child_frame_id_;
        msg.transform = tf2::toMsg(t_map_base.transform());
        tf_message.transforms.emplace_back(msg);
      }

      return tf_message;
    }

    tf2_msgs::msg::TFMessage to_cameras_tf_message(
      std_msgs::msg::Header::_stamp_type stamp,
      const Observations &observations,
      const std::vector<TransformWithCovariance> &t_map_cameras)
    {
      tf2_msgs::msg::TFMessage tf_message;

      geometry_msgs::msg::TransformStamped msg;
      msg.header.stamp = stamp;
      msg.header.frame_id = psl_cxt_.psl_pub_map_frame_id_;

      if (!psl_cxt_.psl_pub_tf_camera_per_marker_child_frame_id_.empty()) {

        for (size_t i = 0; i < observations.size(); i += 1) {
          auto &observation = observations.observations()[i];
          auto &t_map_camera = t_map_cameras[i];

          if (t_map_camera.is_valid()) {
            std::ostringstream oss_child_frame_id;
            oss_child_frame_id << psl_cxt_.psl_pub_tf_camera_per_marker_child_frame_id_
                               << std::setfill('0') << std::setw(3)
                               << observation.id();
            msg.child_frame_id = oss_child_frame_id.str();
            msg.transform = tf2::toMsg(t_map_camera.transform());
            tf_message.transforms.emplace_back(msg);
          }
        }
      }

      return tf_message;
    }

    tf2_msgs::msg::TFMessage to_markers_tf_message(
      std_msgs::msg::Header::_stamp_type stamp,
      const Observations &observations,
      const std::vector<TransformWithCovariance> &t_map_markers)
    {
      tf2_msgs::msg::TFMessage tf_message;

      geometry_msgs::msg::TransformStamped msg;
      msg.header.stamp = stamp;
      msg.header.frame_id = psl_cxt_.psl_pub_map_frame_id_;

      if (!psl_cxt_.psl_pub_tf_marker_per_marker_child_frame_id_.empty()) {

        for (size_t i = 0; i < observations.size(); i += 1) {
          auto &observation = observations.observations()[i];
          auto &t_map_marker = t_map_markers[i];

          if (t_map_marker.is_valid()) {
            std::ostringstream oss_child_frame_id;
            oss_child_frame_id << psl_cxt_.psl_pub_tf_marker_per_marker_child_frame_id_
                               << std::setfill('0') << std::setw(3)
                               << observation.id();
            msg.child_frame_id = oss_child_frame_id.str();
            msg.transform = tf2::toMsg(t_map_marker.transform());
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
        t_map_cameras.emplace_back(t_map_camera);
      }

      return t_map_cameras;
    }

    std::vector<TransformWithCovariance> markers_t_map_markers(
      const Observations &observations,
      const CameraInfoInterface &camera_info,
      double marker_length,
      const TransformWithCovariance &t_map_camera)
    {
      std::vector<TransformWithCovariance> t_map_markers;

      for (const auto &observation : observations.observations()) {
        // someday pay attention to the sam_not_cv flag. For now just use opencv.
        auto t_camera_marker = CvUtils::solve_t_camera_marker(observation, camera_info, marker_length);
        t_map_markers.emplace_back(TransformWithCovariance(t_map_camera.transform() * t_camera_marker.transform()));
      }

      return t_map_markers;
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

  };

  std::shared_ptr<rclcpp::Node> vloc_node_factory(const rclcpp::NodeOptions &options)
  {
    return std::shared_ptr<rclcpp::Node>(new VlocNode(options));
  }
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(fiducial_vlam::VlocNode)
