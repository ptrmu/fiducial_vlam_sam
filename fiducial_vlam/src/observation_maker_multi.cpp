
#include <memory>

#include "fiducial_vlam_msgs/msg/observations_synced.hpp"
#include "fvlam/camera_info.hpp"
#include "fvlam/logger.hpp"
#include "fvlam/observation.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "observation_maker.hpp"
#include "vloc_context.hpp"

namespace fiducial_vlam
{
// ==============================================================================
// SingleObservationMaker class
// ==============================================================================

  class MultiObservationMaker : public ObservationMakerInterface
  {
    rclcpp::Node &node_;
    fvlam::Logger &logger_;
    VlocContext &cxt_;
    ObservationMakerInterface::OnObservationCallback on_observation_callback_;

    std::unique_ptr<ObservationPublisherInterface> observation_publisher_;

    using approximate_sync_policy = message_filters::sync_policies::ApproximateTime<
      fiducial_vlam_msgs::msg::ObservationsSynced,
      fiducial_vlam_msgs::msg::ObservationsSynced>;

    std::vector<std::string> sub_topics_{};

    std::shared_ptr<message_filters::Subscriber<fiducial_vlam_msgs::msg::ObservationsSynced>> left_sub_;
    std::shared_ptr<message_filters::Subscriber<fiducial_vlam_msgs::msg::ObservationsSynced>> right_sub_;

    std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy> > syncApproximate_;

    void observations_sync(const fiducial_vlam_msgs::msg::ObservationsSynced::SharedPtr m0,
                           const fiducial_vlam_msgs::msg::ObservationsSynced::SharedPtr m1)
    {
      auto stamp = fvlam::Stamp::from(m0->header.stamp);
      auto &camera_frame_id = m0->header.frame_id;

      auto camera_info0 = fvlam::CameraInfo::from(m0->observations_synced[0].camera_info);
      auto camera_info1 = fvlam::CameraInfo::from(m1->observations_synced[0].camera_info);
      auto camera_info_map = fvlam::CameraInfoMap{};
      camera_info_map.emplace(camera_info0.imager_frame_id(), camera_info0);
      camera_info_map.emplace(camera_info1.imager_frame_id(), camera_info1);

      auto observations0 = fvlam::Observations::from(m0->observations_synced[0]);
      auto observations1 = fvlam::Observations::from(m1->observations_synced[0]);
      auto observations_synced = fvlam::ObservationsSynced{stamp, camera_frame_id};
      observations_synced.emplace_back(observations0);
      observations_synced.emplace_back(observations1);

      // Publish these observations so that vmap can listen to them to create maps
      observation_publisher_->publish_observations_synced(camera_info_map, observations_synced);

      // Send the observations to vloc to use for generating location estimates.
      on_observation_callback_(camera_info_map, observations_synced);
    }

  public:
    MultiObservationMaker(rclcpp::Node &node, fvlam::Logger &logger, VlocContext &cxt,
                          const ObservationMakerInterface::OnObservationCallback &on_observation_callback) :
      node_{node}, logger_{logger}, cxt_{cxt},
      on_observation_callback_{on_observation_callback},
      observation_publisher_{make_observation_publisher(node_, logger_, cxt_.loc_pub_observations_topic_)}
    {
      // Split the sub topics to figure out how many vdet_nodes are broadcasting observations_synced messages.
      std::size_t previous{0};
      std::size_t current = cxt.loc_sub_multi_observations_topic_.find(':');
      while (current != std::string::npos) {
        sub_topics_.push_back(cxt.loc_sub_multi_observations_topic_.substr(previous, current - previous));
        previous = current + 1;
        current = cxt.loc_sub_multi_observations_topic_.find(':', previous);
      }
      sub_topics_.push_back(cxt.loc_sub_multi_observations_topic_.substr(previous, current - previous));

      left_sub_ = std::make_shared<message_filters::Subscriber<fiducial_vlam_msgs::msg::ObservationsSynced> >(
        &node, sub_topics_[0]);
      right_sub_ = std::make_shared<message_filters::Subscriber<fiducial_vlam_msgs::msg::ObservationsSynced> >(
        &node, sub_topics_[1]);

      syncApproximate_ = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(
        approximate_sync_policy(10), *left_sub_, *right_sub_);
      syncApproximate_->registerCallback(&MultiObservationMaker::observations_sync, this);
    }

    void report_diagnostics(fvlam::Logger &logger,
                            const rclcpp::Time &end_time) override
    {
      (void) logger;
      (void) end_time;
    }
  };

  std::unique_ptr<ObservationMakerInterface> make_multi_observation_maker(
    VlocContext &cxt,
    rclcpp::Node &node,
    fvlam::Logger &logger,
    const ObservationMakerInterface::OnObservationCallback &on_observation_callback)
  {
    return std::make_unique<MultiObservationMaker>(node, logger, cxt, on_observation_callback);
  }
}
