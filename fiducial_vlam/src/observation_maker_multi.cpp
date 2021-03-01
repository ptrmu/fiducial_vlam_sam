
#include <memory>

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
  public:
    MultiObservationMaker(rclcpp::Node &node, fvlam::Logger &logger, VlocContext &cxt,
                          const ObservationMakerInterface::OnObservationCallback &on_observation_callback)
    {
      
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
