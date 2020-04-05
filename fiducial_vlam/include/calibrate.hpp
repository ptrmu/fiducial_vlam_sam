
#ifndef VLAM_CALIBRATE_HPP
#define VLAM_CALIBRATE_HPP

#include "ros2_shared/context_macros.hpp"
#include "fiducial_math.hpp"

namespace rclcpp
{
  class Logger;
}

namespace fiducial_vlam
{

// ==============================================================================
// CalibrateContext class
// ==============================================================================

#define CAL_ALL_PARAMS \
  CXT_MACRO_MEMBER(cal_compute_on_thread, int, 1)             /* Do heavy-duty computation on a thread. */\
  CXT_MACRO_MEMBER(cal_cmd, std::string, "")                  /* Calibrate command. */\
  CXT_MACRO_MEMBER(cal_aruco_dictionary_id, int, 5)           /* dictionary id  */ \
  CXT_MACRO_MEMBER(cal_squares_x, int, 12)                    /* number of squares in the x direction on the charuco board  */ \
  CXT_MACRO_MEMBER(cal_squares_y, int, 9)                     /* number of squares in the y direction on the charuco board  */ \
  CXT_MACRO_MEMBER(cal_square_length, float, 0.030)           /* length of a square on the charuco board  */ \
  CXT_MACRO_MEMBER(cal_upper_left_white_not_black, int, 0)    /* upper_left_white_not_black  */ \
  CXT_MACRO_MEMBER(cal_marker_length, float, 0.0225)          /* length of a marker on the charuco board  */ \
  CXT_MACRO_MEMBER(cal_cv3_do_corner_refinement, int, 1)      /* OpenCV 3.x argument to detect corners. 0 = false, 1 = true */ \
  CXT_MACRO_MEMBER(cal_cv4_corner_refinement_method, int, 2)  /* OpenCV 4.x argument to detect corners. 0 = none, 1 = subpix, 2 = contour, 3 = apriltag */ \
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

  class CalibrateCameraInterface : public ProcessImageInterface
  {
  public:
    virtual std::string calibrate_camera_cmd(const std::string &cmd) = 0;
  };

  std::unique_ptr<CalibrateCameraInterface> make_calibrate_camera(rclcpp::Logger &logger,
                                                                  const CalibrateContext &cxt);

}
#endif //VLAM_CALIBRATE_HPP
