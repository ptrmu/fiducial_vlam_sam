
#include "cv_bridge/cv_bridge.h"
#include "fiducial_vlam_msgs/msg/map.hpp"
#include "fiducial_vlam_msgs/msg/observations_synced.hpp"
#include "fvlam/camera_info.hpp"
#include "fvlam/localize_camera_interface.hpp"
#include "fvlam/logger.hpp"
#include "fvlam/marker.hpp"
#include "fvlam/observation.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "observation_maker.hpp"
#include "vdet_context.hpp"


namespace fvlam
{
// ==============================================================================
// FiducialMarkerCvContext from method
// ==============================================================================

  template<>
  FiducialMarkerContext FiducialMarkerContext::from<fiducial_vlam::VdetContext>(
    fiducial_vlam::VdetContext &other)
  {
    FiducialMarkerContext cxt{other.det_corner_refinement_method_};
    cxt.border_color_red_ = 0;
    cxt.border_color_green_ = 1.0;
    cxt.border_color_blue_ = 0.;
    return cxt;
  }
}

namespace fiducial_vlam
{
  struct SomDiagnostics
  {
    std::uint64_t sub_camera_info_count_{0};
    std::uint64_t sub_image_raw_count_{0};
    std::uint64_t empty_observations_count_{0};
    std::uint64_t pub_observations_count_{0};
    std::uint64_t pub_image_marked_count_{0};
    rclcpp::Time start_time_;

    explicit SomDiagnostics(const rclcpp::Time &start_time) :
      start_time_{start_time}
    {}

    void report(fvlam::Logger &logger, const rclcpp::Time &end_time);
  };

  struct SmmDiagnostics
  {
    std::uint64_t sub_map_count_{0};
    rclcpp::Time start_time_;

    explicit SmmDiagnostics(const rclcpp::Time &start_time) :
      start_time_{start_time}
    {}

    void report(fvlam::Logger &logger, const rclcpp::Time &end_time);
  };


// ==============================================================================
// ObservationPublisher class
// ==============================================================================

  class ObservationPublisher
  {
    rclcpp::Node &node_;
    fvlam::Logger &logger_;
    VdetContext &cxt_;
    const fvlam::MapEnvironment &map_environment_;
    SomDiagnostics &diagnostics_;

    rclcpp::Publisher<fiducial_vlam_msgs::msg::ObservationsSynced>::SharedPtr pub_observations_{};

  public:
    ObservationPublisher(rclcpp::Node &node, fvlam::Logger &logger, VdetContext &cxt,
                         const fvlam::MapEnvironment &map_environment, SomDiagnostics &diagnostics) :
      node_{node}, logger_{logger}, cxt_{cxt},
      map_environment_{map_environment}, diagnostics_{diagnostics}
    {}

    void publish_observations_synced(const fvlam::CameraInfoMap &camera_info_map,
                                     const fvlam::ObservationsSynced &observations_synced)
    {
      // Start by creating a MapEnvironment
      auto map_environment_msg = map_environment_.to<fiducial_vlam_msgs::msg::MapEnvironment>();

      // Create the ObservationSynced message.
      auto msg = fiducial_vlam_msgs::msg::ObservationsSynced{}
        .set__header(std_msgs::msg::Header{}
                       .set__stamp(observations_synced.stamp().to<builtin_interfaces::msg::Time>())
                       .set__frame_id(observations_synced.camera_frame_id()))
        .set__map_environment(map_environment_msg);

      // Add the observattions
      for (auto &observations : observations_synced) {
        // Find the camera info for these observations.
        auto ci = camera_info_map.find(observations.imager_frame_id());
        if (ci == camera_info_map.end()) {
          continue;
        }

        auto msg_item = fiducial_vlam_msgs::msg::Observations{}
          .set__camera_info(ci->second.to<fiducial_vlam_msgs::msg::CameraInfo>())
          .set__observations(observations.to<std::vector<fiducial_vlam_msgs::msg::Observation>>());

        msg.observations_synced.emplace_back(msg_item);
      }

      // Publish the message.
      if (!pub_observations_) {
        pub_observations_ = node_.create_publisher<fiducial_vlam_msgs::msg::ObservationsSynced>(
          cxt_.det_pub_observations_topic_, 2);
      }
      pub_observations_->publish(msg);
      diagnostics_.pub_observations_count_ += 1;
    }
  };

// ==============================================================================
// SingleObservationMaker class
// ==============================================================================

  class SingleObservationMaker : public ObservationMakerInterface
  {
    rclcpp::Node &node_;
    fvlam::Logger &logger_;
    VdetContext &cxt_;
    fvlam::MapEnvironment map_environment_;
    ObservationMakerInterface::OnObservationCallback on_observation_callback_;

    SomDiagnostics diagnostics_;

    ObservationPublisher observation_publisher_;

    std::unique_ptr<fvlam::FiducialMarkerInterface> fiducial_marker_{};

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_raw_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_camera_info_;

    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg_{};
    std_msgs::msg::Header::_stamp_type last_image_stamp_{};

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_marked_{};

  public:
    SingleObservationMaker(rclcpp::Node &node, fvlam::Logger &logger, VdetContext &cxt,
                           const fvlam::MapEnvironment &map_environment,
                           const ObservationMakerInterface::OnObservationCallback &on_observation_callback) :
      node_{node}, logger_{logger}, cxt_{cxt}, map_environment_{map_environment},
      on_observation_callback_{on_observation_callback},
      diagnostics_{node.now()},
      observation_publisher_{node, logger, cxt, map_environment_, diagnostics_}
    {
      // Initialize work objects after parameters have been loaded.
      auto fiducial_marker_context = fvlam::FiducialMarkerContext::from(cxt_);
      fiducial_marker_ = make_fiducial_marker(fiducial_marker_context, map_environment_, logger_);

      // ROS subscriptions
      auto camera_info_qos = cxt_.det_sub_camera_info_best_effort_not_reliable_ ?
                             rclcpp::QoS{rclcpp::SensorDataQoS(rclcpp::KeepLast(1))} :
                             rclcpp::QoS{rclcpp::ServicesQoS()};
      sub_camera_info_ = node.create_subscription<sensor_msgs::msg::CameraInfo>(
        cxt_.det_sub_camera_info_topic_,
        camera_info_qos,
        [this](sensor_msgs::msg::CameraInfo::SharedPtr msg) -> void
        {
          // Save this message because it will be used for the next image message.
          // Note: We are holding on to a shared pointer. This will be released when
          // the next camera_info message is received. Due to the timing of image_raw
          // and camera_info messages, this message may be used for processing zero,
          // one, or two image_raw messages.
          camera_info_msg_ = msg;
          diagnostics_.sub_camera_info_count_ += 1;
        });

      auto image_raw_qos = cxt_.det_sub_image_raw_best_effort_not_reliable_ ?
                           rclcpp::QoS{rclcpp::SensorDataQoS(rclcpp::KeepLast(1))} :
                           rclcpp::QoS{rclcpp::ServicesQoS()};
      sub_image_raw_ = node.create_subscription<sensor_msgs::msg::Image>(
        cxt_.det_sub_image_raw_topic_,
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

            process_image(std::move(msg), *camera_info_msg_);
          }

          last_image_stamp_ = stamp;
          diagnostics_.sub_image_raw_count_ += 1;
        });
    }

    void report_diagnostics(fvlam::Logger &logger,
                            const rclcpp::Time &end_time) override
    {
      (void) logger;
      (void) end_time;
    }

  private:
    void process_image(sensor_msgs::msg::Image::UniquePtr image_msg,
                       sensor_msgs::msg::CameraInfo &sensor_ci_msg)
    {
      auto stamp = fvlam::Stamp::from(image_msg->header.stamp);

      // Convert ROS to OpenCV
      cv_bridge::CvImagePtr gray{cv_bridge::toCvCopy(*image_msg, "mono8")};

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

      // Detect the markers in this image and create a list of
      // observations.
      auto imager_frame_id = cxt_.det_pub_imager_frame_id_.empty() ?
                             image_msg->header.frame_id : cxt_.det_pub_imager_frame_id_;
      auto observations = fiducial_marker_->detect_markers(gray->image, imager_frame_id);

      auto camera_frame_id = cxt_.det_pub_camera_frame_id_.empty() ?
                             image_msg->header.frame_id : cxt_.det_pub_camera_frame_id_;
      auto observations_synced = fvlam::ObservationsSynced(stamp, camera_frame_id);
      observations_synced.emplace_back(observations);

      if (observations.empty()) {
        diagnostics_.empty_observations_count_ += 1;
      }

      // Debugging hint: If the markers in color_marked are not outlined
      // in green, then they haven't been detected.

      // Annotate the image_marked with the markers that were found.
      if (cxt_.det_pub_image_marked_enable_) {

        fiducial_marker_->annotate_image_with_detected_markers(color_marked.image, observations);

        // The marking has been happening on the original message.
        auto image_marked_frame_id = cxt_.det_pub_image_marked_frame_id_.empty() ?
                                     image_msg->header.frame_id : cxt_.det_pub_image_marked_frame_id_;
        image_msg->header.frame_id = image_marked_frame_id;

        // Republish it now.
        if (!pub_image_marked_) {
          pub_image_marked_ = node_.create_publisher<sensor_msgs::msg::Image>(
            cxt_.det_pub_image_marked_topic_, 2);
        }
        pub_image_marked_->publish(std::move(image_msg));
        diagnostics_.pub_image_marked_count_ += 1;
      }

      // Create our CameraInfo message by first creating a CameraInfo structure
      // from a ROS2 CameraIndo message.
      auto t_camera_imager = fvlam::Transform3{
        fvlam::Rotate3::RzRyRx(cxt_.det_t_camera_imager_yaw_,
                               cxt_.det_t_camera_imager_pitch_,
                               cxt_.det_t_camera_imager_roll_),
        fvlam::Translate3{cxt_.det_t_camera_imager_x_,
                          cxt_.det_t_camera_imager_y_,
                          cxt_.det_t_camera_imager_z_}};
      auto camera_info = fvlam::CameraInfo{imager_frame_id,
                                           fvlam::CameraInfo::from(sensor_ci_msg),
                                           t_camera_imager};
      auto camera_info_map = fvlam::CameraInfoMap{};
      camera_info_map.emplace(camera_info.imager_frame_id(), camera_info);


      // publish the observations if requested
      if (cxt_.det_pub_observations_enable_) {
        observation_publisher_.publish_observations_synced(camera_info_map, observations_synced);
      }

      // Callback with the observations.
      on_observation_callback_(camera_info_map, observations_synced);
    }
  };

  std::unique_ptr<ObservationMakerInterface> make_single_observation_maker(
    VdetContext &cxt,
    rclcpp::Node &node,
    fvlam::Logger &logger,
    const fvlam::MapEnvironment &map_environment,
    const ObservationMakerInterface::OnObservationCallback &on_observation_callback)
  {
    return std::make_unique<SingleObservationMaker>(node, logger, cxt, map_environment, on_observation_callback);
  }

// ==============================================================================
// MarkerMapSubscriber class
// ==============================================================================

  class MarkerMapSubscriber : public MarkerMapSubscriberInterface
  {
    rclcpp::Node &node_;
    fvlam::Logger &logger_;
    VdetContext &cxt_;
    MarkerMapSubscriberInterface::OnMapEnvironmentChanged on_map_environment_changed_;

    SmmDiagnostics diagnostics_;
    fvlam::MarkerMap marker_map_{};
    fvlam::MapEnvironment map_environment_{};

    rclcpp::Subscription<fiducial_vlam_msgs::msg::Map>::SharedPtr sub_map_{};

  public:
    MarkerMapSubscriber(rclcpp::Node &node, fvlam::Logger &logger, VdetContext &cxt,
                        const MarkerMapSubscriberInterface::OnMapEnvironmentChanged &on_map_environment_changed) :
      node_{node}, logger_{logger}, cxt_{cxt},
      on_map_environment_changed_{on_map_environment_changed},
      diagnostics_{node.now()}
    {
      sub_map_ = node.create_subscription<fiducial_vlam_msgs::msg::Map>(
        cxt_.det_sub_map_topic_,
        1,
        [this](const fiducial_vlam_msgs::msg::Map::UniquePtr msg) -> void
        {
          marker_map_ = fvlam::MarkerMap::from(*msg);
          diagnostics_.sub_map_count_ += 1;
          if (!map_environment_.equals(marker_map_.map_environment())) {
            map_environment_ = marker_map_.map_environment();
            on_map_environment_changed_(marker_map_.map_environment());
          }
        });
    }

    fvlam::MarkerMap const &marker_map() const override
    {
      return marker_map_;
    }

    void report_diagnostics(fvlam::Logger &logger,
                            const rclcpp::Time &end_time) override
    {
      (void) logger;
      (void) end_time;
    }
  };

  std::unique_ptr<MarkerMapSubscriberInterface> make_marker_map_subscriber(
    VdetContext &cxt,
    rclcpp::Node &node,
    fvlam::Logger &logger,
    const MarkerMapSubscriberInterface::OnMapEnvironmentChanged &on_map_environment_changed)
  {
    return std::make_unique<MarkerMapSubscriber>(node, logger, cxt, on_map_environment_changed);
  }

}
