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
  class ObservationsSynced;//
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
    ObservationMakerInterface::OnObservationCallback on_observation_callback);
}