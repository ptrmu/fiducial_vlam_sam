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
  class MarkerMap; //
}

namespace fiducial_vlam
{
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

  template<class TContext>
  std::unique_ptr<ObservationMakerInterface> make_observation_maker(
    TContext &cxt,
    rclcpp::Node &node,
    fvlam::Logger &logger,
    const ObservationMakerInterface::OnObservationCallback &on_observation_callback);

// ==============================================================================
// MarkerMapSubscriberInterface class
// ==============================================================================

  class MarkerMapSubscriberInterface
  {
  public:
    using OnMapEnvironmentChanged = std::function<void(const fvlam::MarkerMap &marker_map)>;

    virtual ~MarkerMapSubscriberInterface() = default;

    virtual fvlam::MarkerMap const &marker_map() const = 0;

    virtual void report_diagnostics(fvlam::Logger &logger,
                                    const rclcpp::Time &end_time) = 0;
  };

  template<class TContext>
  std::unique_ptr<MarkerMapSubscriberInterface> make_marker_map_subscriber(
    TContext &cxt,
    rclcpp::Node &node,
    fvlam::Logger &logger,
    const MarkerMapSubscriberInterface::OnMapEnvironmentChanged &on_map_environment_changed);
}