
#include "calibrate.hpp"

#include "fiducial_math.hpp"
#include "transform_with_covariance.hpp"

namespace fiducial_vlam
{

// ==============================================================================
// CalibrateCameraImpl class
// ==============================================================================

  class CalibrateCameraImpl : public CalibrateCameraInterface
  {
    const CalibrateContext &cal_cxt_;

  public:
    explicit CalibrateCameraImpl(const CalibrateContext &cal_cxt) :
      cal_cxt_{cal_cxt}
    {}

    TransformWithCovariance solve_t_map_camera(const Observations &observations,
                                               const CameraInfoInterface &camera_info,
                                               const Map &map) override
    {
      return TransformWithCovariance{};
    }

    std::string calibrate_camera_cmd(const std::string cmd) override
    {
      return std::string{};
    }
  };

  std::unique_ptr<CalibrateCameraInterface> make_calibrate_camera(const CalibrateContext &cxt)
  {
    return std::make_unique<CalibrateCameraImpl>(cxt);
  }

}
