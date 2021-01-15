
#include "fvlam/localize_camera_interface.hpp"
#include "fvlam/observation.hpp"

namespace fvlam
{

// ==============================================================================
// LocalizeCameraGtsam class
// ==============================================================================

  class LocalizeCameraGtsam : public LocalizeCameraInterface
  {
  public:
    LocalizeCameraGtsam(const LocalizeCameraGtsamContext &lc_context, Logger &logger)
    {}

    // Given observations of fiducial markers and a map of world locations of those
    // markers, figure out the camera pose in the world frame.
    Transform3WithCovariance solve_t_map_camera(const Observations &observations,
                                                const CameraInfo &camera_info,
                                                const MarkerMap &map) override
    {

    }

    // Given the corners of one marker (observation) calculate t_camera_marker.
    Transform3WithCovariance solve_t_camera_marker(const Observation &observation,
                                                   const CameraInfo &camera_info,
                                                   double marker_length) override
    {

    }
  };

  template<>
  std::unique_ptr<LocalizeCameraInterface> make_localize_camera<LocalizeCameraGtsamContext>(
    const LocalizeCameraGtsamContext &lc_context, Logger &logger)
  {
    return std::make_unique<LocalizeCameraGtsam>(lc_context, logger);
  }

}
