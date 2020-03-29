
#ifndef VLAM_CALIBRATE_HPP
#define VLAM_CALIBRATE_HPP

#include "ros2_shared/context_macros.hpp"
#include "fiducial_math.hpp"

namespace fiducial_vlam
{

// ==============================================================================
// CalibrateContext class
// ==============================================================================

#define CAL_ALL_PARAMS \
  CXT_MACRO_MEMBER(       /* Do heavy-duty computation on a thread. */ \
  cal_compute_on_thread, \
  int, 1) \
  CXT_MACRO_MEMBER(       /* Calibrate command. */ \
  cal_calibrate_camera_cmd, \
  std::string, "") \
  /* End of list */

  struct CalibrateContext
  {
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)
    CAL_ALL_PARAMS
  };

// ==============================================================================
// CalibrateCameraInterface class
// ==============================================================================

  class CalibrateCameraInterface : public LocalizeCameraInterface
  {
  public:
    virtual std::string calibrate_camera_cmd(const std::string cmd) = 0;
  };

  std::unique_ptr<CalibrateCameraInterface> make_calibrate_camera(const CalibrateContext &cxt);

}
#endif //VLAM_CALIBRATE_HPP
