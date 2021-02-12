#pragma ide diagnostic ignored "modernize-use-nodiscard"

#include <memory>

#include "fvlam/build_marker_map_interface.hpp"
#include "fvlam/observations_bundle.hpp"

namespace fvlam
{

// ==============================================================================
// BuildMarkerMapRecorder class
// ==============================================================================

  class BuildMarkerMapRecorder : public BuildMarkerMapInterface
  {
    BuildMarkerMapRecorderContext recorder_context_;
    Logger &logger_;
    std::unique_ptr<fvlam::MarkerMap> map_initial_;
    ObservationsBundles observations_bundles_;

  public:
    BuildMarkerMapRecorder(BuildMarkerMapRecorderContext recorder_context,
                           Logger &logger,
                           MarkerMap map_initial) :
      recorder_context_{std::move(recorder_context)}, logger_{logger},
      map_initial_{std::make_unique<fvlam::MarkerMap>(map_initial)}, observations_bundles_{std::move(map_initial)}
    {}

    ~BuildMarkerMapRecorder() override// virtual destructor
    {
      observations_bundles_.save(recorder_context_.bmm_recorded_observations_name_, logger_);
    }

    // Take the location of markers in one image and add them to the marker map
    // building algorithm.
    void process(const fvlam::Observations &observations,
                 const fvlam::CameraInfo &camera_info) override
    {
      observations_bundles_.add_bundle(ObservationsBundle{camera_info, observations});
    }

    // Given the observations that have been added so far, create and return a marker_map.
    std::unique_ptr<fvlam::MarkerMap> build() override
    {
      return std::make_unique<fvlam::MarkerMap>(*map_initial_);
    }
  };

  template<>
  std::unique_ptr<BuildMarkerMapInterface> make_build_marker_map<BuildMarkerMapRecorderContext>(
    const BuildMarkerMapRecorderContext &bmm_context,
    Logger &logger,
    const MarkerMap &map_initial)
  {
    return std::make_unique<BuildMarkerMapRecorder>(bmm_context, logger, map_initial);
  }

}
