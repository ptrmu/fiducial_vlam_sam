
#include "cv_bridge/cv_bridge.h"
#include "fiducial_vlam/fiducial_vlam.hpp"
#include "fiducial_vlam_msgs/msg/observations.hpp"
#include "fvlam/localize_camera_interface.hpp"
#include "fvlam/logger.hpp"
#include "fvlam/observation.hpp"
#include <gtsam/base/timing.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "logger_ros2.hpp"
#include "vdet_context.hpp"

namespace fvlam
{

// ==============================================================================
// FiducialMarkerCvContext from method
// ==============================================================================

  template<>
  FiducialMarkerCvContext FiducialMarkerCvContext::from<fiducial_vlam::VdetContext>(
    fiducial_vlam::VdetContext &other)
  {
    FiducialMarkerCvContext cxt{other.det_corner_refinement_method_};
    cxt.border_color_red_ = 0;
    cxt.border_color_green_ = 1.0;
    cxt.border_color_blue_ = 0.;
    cxt.aruco_dictionary_id_ = other.det_aruco_dictionary_id_;
    return cxt;
  }
}

namespace fiducial_vlam
{
// ==============================================================================
// PsdContext class
// ==============================================================================

  struct PsdContext
  {
#undef PAMA_PARAM
#define PAMA_PARAM(n, t, d) PAMA_PARAM_DEFINE(n, t, d)
    PAMA_PARAMS_DEFINE(PSD_ALL_PARAMS)
  };

// ==============================================================================
// VdetNode class
// ==============================================================================

  class VdetNode : public rclcpp::Node
  {
    LoggerRos2 logger_;
    VdetDiagnostics diagnostics_;

    VdetContext cxt_{};
    PsdContext psd_cxt_{};

    std::unique_ptr<fvlam::FiducialMarkerInterface> fiducial_marker_{};

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_raw_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_camera_info_;

    std::unique_ptr<sensor_msgs::msg::CameraInfo> camera_info_msg_{};
    std_msgs::msg::Header::_stamp_type last_image_stamp_{};

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_marked_{};
    rclcpp::Publisher<fiducial_vlam_msgs::msg::Observations>::SharedPtr pub_observations_{};

    void validate_parameters()
    {}

    void validate_psd_parameters()
    {}

    void setup_parameters()
    {
#undef PAMA_PARAM
#define PAMA_PARAM(n, t, d) PAMA_PARAM_INIT(n, t, d)
      PAMA_PARAMS_INIT((*this), cxt_, , VDET_ALL_PARAMS, validate_parameters)
      PAMA_PARAMS_INIT((*this), psd_cxt_, , PSD_ALL_PARAMS, validate_psd_parameters)

#undef PAMA_PARAM
#define PAMA_PARAM(n, t, d) PAMA_PARAM_CHANGED(n, t, d)
      PAMA_PARAMS_CHANGED((*this), cxt_, , VDET_ALL_PARAMS, validate_parameters, RCLCPP_INFO)
      PAMA_PARAMS_CHANGED((*this), psd_cxt_, , PSD_ALL_PARAMS, validate_psd_parameters, RCLCPP_INFO)

#undef PAMA_PARAM
#define PAMA_PARAM(n, t, d) PAMA_PARAM_LOG(n, t, d)
      PAMA_PARAMS_LOG((*this), cxt_, , VDET_ALL_PARAMS, RCLCPP_INFO)
      PAMA_PARAMS_LOG((*this), psd_cxt_, , PSD_ALL_PARAMS, RCLCPP_INFO)

#undef PAMA_PARAM
#define PAMA_PARAM(n, t, d) PAMA_PARAM_CHECK_CMDLINE(n, t, d)
      PAMA_PARAMS_CHECK_CMDLINE((*this), , VDET_ALL_PARAMS PSD_ALL_PARAMS, RCLCPP_ERROR)
    }

  public:
    explicit VdetNode(const rclcpp::NodeOptions &options) :
      Node{"vdet_node", options},
      logger_{*this},
      diagnostics_{now()}
    {
      // Get parameters from the command line
      setup_parameters();

      // Initialize work objects after parameters have been loaded.
      auto fiducial_marker_context = fvlam::FiducialMarkerCvContext::from(cxt_);
      fiducial_marker_ = make_fiducial_marker(fiducial_marker_context, logger_);

      // ROS subscriptions
      auto camera_info_qos = psd_cxt_.psd_sub_camera_info_best_effort_not_reliable_ ?
                             rclcpp::QoS{rclcpp::SensorDataQoS(rclcpp::KeepLast(1))} :
                             rclcpp::QoS{rclcpp::ServicesQoS()};
      sub_camera_info_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        psd_cxt_.psd_sub_camera_info_topic_,
        camera_info_qos,
        [this](sensor_msgs::msg::CameraInfo::UniquePtr msg) -> void
        {
          // Save this message because it will be used for the next image message.
          // Note: we are taking ownership of this object
          camera_info_msg_ = std::move(msg);
          diagnostics_.sub_camera_info_count_ += 1;
        });

      auto image_raw_qos = psd_cxt_.psd_sub_image_raw_best_effort_not_reliable_ ?
                           rclcpp::QoS{rclcpp::SensorDataQoS(rclcpp::KeepLast(1))} :
                           rclcpp::QoS{rclcpp::ServicesQoS()};
      sub_image_raw_ = create_subscription<sensor_msgs::msg::Image>(
        psd_cxt_.psd_sub_image_raw_topic_,
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

      RCLCPP_INFO(get_logger(), "vdet_node ready");
    }


  private:
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

      if (psd_cxt_.psd_pub_image_marked_enable_) {

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

//        // Change image_marked to use the standard parent frame
//        image_msg->header.frame_id = psd_cxt_.psd_pub_map_frame_id_;
      }
      gttoc(grey);

      gttic(color_marked);
      // Detect the markers in this image and create a list of
      // observations.
      auto observations = fiducial_marker_->detect_markers(gray->image);
      gttoc(color_marked);

      if (observations.empty()) {
        diagnostics_.empty_observations_count_ += 1;
      }

      gttic(observations);
      // Annotate the image_marked with the markers that were found.
      if (psd_cxt_.psd_pub_image_marked_enable_) {
        fiducial_marker_->annotate_image_with_detected_markers(color_marked.image, observations);
      }

      // Publish the observations.
      if (psd_cxt_.psd_pub_observations_enable_ &&
          !observations.empty()) {
        auto msg = fiducial_vlam_msgs::msg::Observations{}
          .set__header(image_msg->header)
          .set__camera_info(*camera_info_msg)
          .set__observations(observations.to<std::vector<fiducial_vlam_msgs::msg::Observation>>());
        if (!pub_observations_) {
          pub_observations_ = create_publisher<fiducial_vlam_msgs::msg::Observations>(
            psd_cxt_.psd_pub_observations_topic_, 2);
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

      // Publish an annotated image if requested. Even if there is no map.
      if (psd_cxt_.psd_pub_image_marked_enable_) {
        // The marking has been happening on the original message.
        // Republish it now.
        if (!pub_image_marked_) {
          pub_image_marked_ = create_publisher<sensor_msgs::msg::Image>(
            psd_cxt_.psd_pub_image_marked_topic_, 2);
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


  };

  std::shared_ptr<rclcpp::Node> vdet_node_factory(const rclcpp::NodeOptions &options)
  {
    return std::shared_ptr<rclcpp::Node>(new VdetNode(options));
  }
}
