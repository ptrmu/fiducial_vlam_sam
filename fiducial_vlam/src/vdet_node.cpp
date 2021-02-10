
#include "cv_bridge/cv_bridge.h"
#include "fiducial_vlam/fiducial_vlam.hpp"
#include "fiducial_vlam_msgs/msg/observations_stamped.hpp"
#include "fvlam/camera_info.hpp"
#include "fvlam/localize_camera_interface.hpp"
#include "fvlam/logger.hpp"
#include "fvlam/marker.hpp"
#include "fvlam/observation.hpp"
#include <gtsam/base/timing.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "logger_ros2.hpp"
#include "observation_maker.hpp"
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

  struct VdetDiagnostics
  {
    std::uint64_t sub_camera_info_count_{0};
    std::uint64_t sub_image_raw_count_{0};
    std::uint64_t empty_observations_count_{0};
    std::uint64_t pub_observations_count_{0};
    std::uint64_t pub_image_marked_count_{0};
    rclcpp::Time start_time_;

    explicit VdetDiagnostics(const rclcpp::Time &start_time) :
      start_time_{start_time}
    {}

    void report(fvlam::Logger &logger, const rclcpp::Time &end_time);
  };

// ==============================================================================
// VdetNode class
// ==============================================================================

  class VdetNode : public rclcpp::Node
  {
    LoggerRos2 logger_;
    VdetDiagnostics diagnostics_;

    VdetContext cxt_{};

    std::unique_ptr<ObservationMakerInterface> observation_maker_{};

    rclcpp::Publisher<fiducial_vlam_msgs::msg::ObservationsStamped>::SharedPtr pub_observations_{};

    void validate_parameters()
    {}

    void setup_parameters()
    {
#undef PAMA_PARAM
#define PAMA_PARAM(n, t, d) PAMA_PARAM_INIT(n, t, d)
      PAMA_PARAMS_INIT((*this), cxt_, , VDET_ALL_PARAMS, validate_parameters)

#undef PAMA_PARAM
#define PAMA_PARAM(n, t, d) PAMA_PARAM_CHANGED(n, t, d)
      PAMA_PARAMS_CHANGED((*this), cxt_, , VDET_ALL_PARAMS, validate_parameters, RCLCPP_INFO)

#undef PAMA_PARAM
#define PAMA_PARAM(n, t, d) PAMA_PARAM_LOG(n, t, d)
      PAMA_PARAMS_LOG((*this), cxt_, , VDET_ALL_PARAMS, RCLCPP_INFO)

#undef PAMA_PARAM
#define PAMA_PARAM(n, t, d) PAMA_PARAM_CHECK_CMDLINE(n, t, d)
      PAMA_PARAMS_CHECK_CMDLINE((*this), , VDET_ALL_PARAMS, RCLCPP_ERROR)
    }

  public:
    explicit VdetNode(const rclcpp::NodeOptions &options) :
      Node{"vdet_node", options},
      logger_{*this},
      diagnostics_{now()}
    {
      // Get parameters from the command line
      setup_parameters();

      observation_maker_ = make_observation_maker(
        cxt_, *this, logger_,
        [this](const fvlam::CameraInfoMap &camera_info_map,
               const fvlam::ObservationsSynced &observations_synced) -> void
        {

          // Create an ObservationStamped message.
          // Start by creating a MapEnvironment
          auto map_environment_msg = fvlam::MapEnvironment{
            cxt_.det_map_description_, cxt_.det_aruco_dictionary_id_, cxt_.det_marker_length_}
            .to<fiducial_vlam_msgs::msg::MapEnvironment>();

          // Then create our CameraInfo message.
          auto camera_info_msg = camera_info_map.begin()->second.to<fiducial_vlam_msgs::msg::CameraInfo>();

          // Then bundle up the observations and the CameraInfo message
          auto observations_msg = fiducial_vlam_msgs::msg::Observations{}
            .set__camera_info(camera_info_msg)
            .set__observations((*observations_synced.begin()).to<std::vector<fiducial_vlam_msgs::msg::Observation>>());

          // Finally create the ObservationsStamped message. Use the image message
          // stamp and the frame_id as specified by the parameters.
          auto msg = fiducial_vlam_msgs::msg::ObservationsStamped{}
            .set__header(std_msgs::msg::Header{}
//                           .set__stamp(observations_synced.stamp()) // ToDo fix this
                           .set__frame_id(observations_synced.frame_id()))
            .set__map_environment(map_environment_msg)
            .set__observations(observations_msg);

          if (!pub_observations_) {
            pub_observations_ = create_publisher<fiducial_vlam_msgs::msg::ObservationsStamped>(
              cxt_.det_pub_observations_topic_, 2);
          }
          pub_observations_->publish(msg);
          diagnostics_.pub_observations_count_ += 1;
        });

      RCLCPP_INFO(get_logger(), "vdet_node ready");
    }


  private:
#if 0
    void process_image(sensor_msgs::msg::Image::UniquePtr image_msg,
                       sensor_msgs::msg::CameraInfo &sensor_ci_msg)
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

      if (cxt_.det_pub_image_marked_enable_) {

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
//        image_msg->header.frame_id = cxt_.det_pub_map_frame_id_;
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
      if (cxt_.det_pub_image_marked_enable_) {
        fiducial_marker_->annotate_image_with_detected_markers(color_marked.image, observations);
      }

      // Publish the observations.
      if (cxt_.det_pub_observations_enable_ &&
          !observations.empty()) {

        // Create an ObservationStamped message.
        // Start by creating a MapEnvironment
        auto map_environment_msg = fvlam::MapEnvironment{
          cxt_.det_map_description_, cxt_.det_aruco_dictionary_id_, cxt_.det_marker_length_}
          .to<fiducial_vlam_msgs::msg::MapEnvironment>();

        // Then create our CameraInfo message by first creating a CameraInfo structure
        // from a ROS2 CameraIndo message.
        auto camera_info_frame_id = cxt_.det_pub_camera_info_frame_id_.empty() ?
                                    image_msg->header.frame_id : cxt_.det_pub_camera_info_frame_id_;
        auto t_base_camera = fvlam::Transform3{
          fvlam::Rotate3::RzRyRx(cxt_.det_t_base_camera_yaw_,
                                 cxt_.det_t_base_camera_pitch_,
                                 cxt_.det_t_base_camera_roll_),
          fvlam::Translate3{cxt_.det_t_base_camera_x_,
                            cxt_.det_t_base_camera_y_,
                            cxt_.det_t_base_camera_z_}};
        auto camera_info = fvlam::CameraInfo{camera_info_frame_id,
                                             fvlam::CameraInfo::from(sensor_ci_msg),
                                             t_base_camera};
        auto camera_info_msg = camera_info.to<fiducial_vlam_msgs::msg::CameraInfo>();

        // Then bundle up the observations and the CameraInfo message
        auto observations_msg = fiducial_vlam_msgs::msg::Observations{}
          .set__camera_info(camera_info_msg)
          .set__observations(observations.to<std::vector<fiducial_vlam_msgs::msg::Observation>>());

        // Finally create the ObservationsStamped message. Use the image message
        // stamp and the frame_id as specified by the parameters.
        auto observations_frame_id = cxt_.det_pub_observations_frame_id_.empty() ?
                                     image_msg->header.frame_id : cxt_.det_pub_observations_frame_id_;
        auto msg = fiducial_vlam_msgs::msg::ObservationsStamped{}
          .set__header(std_msgs::msg::Header{}
                         .set__stamp(image_msg->header.stamp)
                         .set__frame_id(observations_frame_id))
          .set__map_environment(map_environment_msg)
          .set__observations(observations_msg);

        if (!pub_observations_) {
          pub_observations_ = create_publisher<fiducial_vlam_msgs::msg::ObservationsStamped>(
            cxt_.det_pub_observations_topic_, 2);
        }
        pub_observations_->publish(msg);
        diagnostics_.pub_observations_count_ += 1;
      }

      // Debugging hint: If the markers in color_marked are not outlined
      // in green, then they haven't been detected.

      // Publish an annotated image if requested.
      if (cxt_.det_pub_image_marked_enable_) {
        // The marking has been happening on the original message.
        auto image_marked_frame_id = cxt_.det_pub_image_marked_frame_id_.empty() ?
                                     image_msg->header.frame_id : cxt_.det_pub_image_marked_frame_id_;
        image_msg->header.frame_id = image_marked_frame_id;

        // Republish it now.
        if (!pub_image_marked_) {
          pub_image_marked_ = create_publisher<sensor_msgs::msg::Image>(
            cxt_.det_pub_image_marked_topic_, 2);
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

  };

  std::shared_ptr<rclcpp::Node> vdet_node_factory(const rclcpp::NodeOptions &options)
  {
    return std::shared_ptr<rclcpp::Node>(new VdetNode(options));
  }
}
