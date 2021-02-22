#pragma ide diagnostic ignored "modernize-use-nodiscard"

#include <memory>

#include "fvlam/build_marker_map_interface.hpp"
#include "fvlam/observations_series.hpp"

namespace fvlam
{

// ==============================================================================
// BuildMarkerMapRecorder class
// ==============================================================================

  class BuildMarkerMapRecorder : public BuildMarkerMapInterface
  {
    BuildMarkerMapRecorderContext recorder_context_;
    Logger &logger_;
    MarkerMap map_initial_;
    CameraInfoMap camera_info_map_;
    std::vector<ObservationsSynced> observations_synced_list_{};

  public:
    BuildMarkerMapRecorder(BuildMarkerMapRecorderContext recorder_context,
                           Logger &logger,
                           MarkerMap map_initial) :
      recorder_context_{std::move(recorder_context)}, logger_{logger},
      map_initial_{std::move(map_initial)},
      camera_info_map_{},
      observations_synced_list_{}
    {}

    ~BuildMarkerMapRecorder() override// virtual destructor
    {
      ObservationsSeries os{std::move(map_initial_), std::move(camera_info_map_),
                            std::move(observations_synced_list_)};
      os.save(recorder_context_.bmm_recorded_observations_name_, logger_);
    }

    // Take the location of markers in one image and add them to the marker map
    // building algorithm.
    void process(const ObservationsSynced &observations_synced,
                 const CameraInfoMap &camera_info_map) override
    {
      // Walk through the camera_info_map and add to our map and camera_infos
      // that we don't already know about. Currently all the camera_info_maps will
      // be the same, but that might loosen in the future.
      for (auto &camera_info_pair: camera_info_map) {
        auto camera_info_it = camera_info_map_.find(camera_info_pair.first);
        if (camera_info_it == camera_info_map_.end()) {
          camera_info_map_.emplace(camera_info_pair.first, camera_info_pair.second);
        }
      }

      observations_synced_list_.emplace_back(observations_synced);
    }

    // Given the observations that have been added so far, create and return a marker_map.
    std::unique_ptr<fvlam::MarkerMap> build() override
    {
      return std::make_unique<fvlam::MarkerMap>(map_initial_);
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
