#pragma ide diagnostic ignored "modernize-use-nodiscard"

#include <memory>

#include "fvlam/build_marker_map_interface.hpp"
#include "fvlam/camera_info.hpp"
#include "fvlam/marker.hpp"
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/sfm/ShonanAveraging.h>
#include <gtsam/slam/InitializePose3.h>
#include <opencv2/calib3d/calib3d.hpp>

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

  public:
    BuildMarkerMapRecorder(const BuildMarkerMapRecorderContext &recorder_context,
                           Logger &logger,
                           MarkerMap map_initial) :
      recorder_context_{recorder_context}, logger_{logger}, map_initial_{map_initial}
    {}

    // Take the location of markers in one image and add them to the marker map
    // building algorithm.
    virtual void process(const fvlam::Observations &observations,
                         const fvlam::CameraInfo &camera_info)
    {

    }

    // Given the observations that have been added so far, create and return a marker_map.
    virtual std::unique_ptr<fvlam::MarkerMap> build()
    {
      return std::unique_ptr<fvlam::MarkerMap>{};
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
