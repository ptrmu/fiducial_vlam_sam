#pragma once

#include <functional>
#include <memory>
#include <string>

#include "ros2_shared/param_macros.hpp"

namespace rclcpp
{
  class Time; //
  class Node; //
}

namespace fvlam
{
  class CameraInfoMap;//
  class Logger; //
  class ObservationsSynced; //
  class MapEnvironment; //
  class MarkerMap; //
}

namespace fiducial_vlam
{
  class VdetContext; //
  class VlocContext; //

// ==============================================================================
// ObservationMakerInterface class
// ==============================================================================

  class ObservationMakerInterface
  {
  public:
    using OnObservationCallback = std::function<void(const fvlam::CameraInfoMap &,
                                                     const fvlam::ObservationsSynced &)>;

    virtual ~ObservationMakerInterface() = default;

    virtual void report_diagnostics(fvlam::Logger &logger,
                                    const rclcpp::Time &end_time) = 0;
  };

  std::unique_ptr<ObservationMakerInterface> make_single_observation_maker(
    VdetContext &cxt,
    rclcpp::Node &node,
    fvlam::Logger &logger,
    const fvlam::MapEnvironment &map_environment,
    const ObservationMakerInterface::OnObservationCallback &on_observation_callback);

  std::unique_ptr<ObservationMakerInterface> make_multi_observation_maker(
    VlocContext &cxt,
    rclcpp::Node &node,
    fvlam::Logger &logger,
    const ObservationMakerInterface::OnObservationCallback &on_observation_callback);

  template<class TContext>
  std::unique_ptr<ObservationMakerInterface> make_observation_maker(
    TContext &cxt,
    rclcpp::Node &node,
    fvlam::Logger &logger,
    const ObservationMakerInterface::OnObservationCallback &on_observation_callback);

// ==============================================================================
// ObservationMakerInterface class
// ==============================================================================

  class ObservationPublisherInterface
  {
  public:
    virtual ~ObservationPublisherInterface() = default;

    virtual void publish_observations_synced(const fvlam::CameraInfoMap &camera_info_map,
                                             const fvlam::ObservationsSynced &observations_synced) = 0;

    virtual void report_diagnostics(fvlam::Logger &logger,
                                    const rclcpp::Time &end_time) = 0;
  };

  std::unique_ptr<ObservationPublisherInterface> make_observation_publisher(
    rclcpp::Node &node,
    fvlam::Logger &logger,
    const std::string &pub_observations_synced_topic);

// ==============================================================================
// MarkerMapSubscriberInterface class
// ==============================================================================

  class MarkerMapSubscriberInterface
  {
  public:
    using OnMapEnvironmentChanged = std::function<void(const fvlam::MapEnvironment &map_environment)>;

    virtual ~MarkerMapSubscriberInterface() = default;

    virtual fvlam::MarkerMap const &marker_map() const = 0;

    virtual void report_diagnostics(fvlam::Logger &logger,
                                    const rclcpp::Time &end_time) = 0;
  };

  std::unique_ptr<MarkerMapSubscriberInterface> make_marker_map_subscriber(
    VdetContext &cxt,
    rclcpp::Node &node,
    fvlam::Logger &logger,
    const MarkerMapSubscriberInterface::OnMapEnvironmentChanged &on_map_environment_changed);
}