
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
  class CapturedImages; //
  class ImageHolder; //
  class CalibrateContext; //

// ==============================================================================
// CalibrateContext class
// ==============================================================================

#define CAL_ALL_PARAMS \
  CXT_MACRO_MEMBER(cal_compute_on_thread, int, 1)                 /* Do heavy-duty computation on a thread. */\
  CXT_MACRO_MEMBER(cal_calibration_style_to_save, int, 3)         /* The calibration style to save. */\
  CXT_MACRO_MEMBER(cal_images_file_name, std::string, "cal_img")  /* Calibrate command. */\
  CXT_MACRO_MEMBER(cal_save_camera_info_path, std::string, "camera_info.yaml") /* Name of file to store calibration in. */\
  CXT_MACRO_MEMBER(cal_camera_name, std::string, "Calibrated Camera") /* Name of the calibrated camera. */\
  CXT_MACRO_MEMBER(cal_cmd, std::string, "")                      /* Calibrate command. */\
  CXT_MACRO_MEMBER(cal_aruco_dictionary_id, int, 5)               /* dictionary id  */ \
  CXT_MACRO_MEMBER(cal_squares_x, int, 12)                        /* number of squares in the x direction on the charuco board  */ \
  CXT_MACRO_MEMBER(cal_squares_y, int, 9)                         /* number of squares in the y direction on the charuco board  */ \
  CXT_MACRO_MEMBER(cal_square_length, float, 0.060)               /* length of a square on the charuco board  */ \
  CXT_MACRO_MEMBER(cal_upper_left_white_not_black, int, 0)        /* upper_left_white_not_black  */ \
  CXT_MACRO_MEMBER(cal_marker_length, float, 0.0450)              /* length of a marker on the charuco board  */ \
  CXT_MACRO_MEMBER(cal_stationary_capture_ms, int, 0)             /* Duration that calibration target needs to be stationary. 0 => infinite */ \
  CXT_MACRO_MEMBER(cal_bootstrap_reserve_fraction, double, 0.2)   /* % of calibration images to reserve for bootstrap validation */ \
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
    virtual std::string cal_cmd(const std::string &cmd,
                                const rclcpp::Time &now) = 0; //
    virtual std::string on_timer(const rclcpp::Time &now) = 0; //
    virtual bool calibration_complete() = 0; //
    virtual void get_captured_image_marked(const rclcpp::Time &now,
                                           cv::Mat &captured_image_marked) = 0; //
  };

  std::unique_ptr<CalibrateCameraInterface> make_calibrate_camera(rclcpp::Logger &logger,
                                                                  const CalibrateContext &cxt);

// ==============================================================================
// CalibrateCaptureInterface class
// ==============================================================================

  class CalibrateCaptureInterface
  {
  public:
    virtual ~CalibrateCaptureInterface() = default; //
    virtual void test_capture(std::shared_ptr<ImageHolder> &image_holder,
                              cv::Mat &color_marked) = 0; //
  };

  std::unique_ptr<CalibrateCaptureInterface> make_calibrate_capture_stationary(rclcpp::Logger &logger,
                                                                               const CalibrateContext &cxt,
                                                                               const rclcpp::Time &now,
                                                                               CapturedImages &captured_images);

// ==============================================================================
// CalibrateCameraTaskInterface class
// ==============================================================================

  struct CalibrateCameraTaskResult
  {
    std::string calibration_report_{};
    std::vector<cv::Mat> captured_images_marked_{};
  };

  class CalibrateCameraTaskInterface
  {
  public:
    virtual ~CalibrateCameraTaskInterface() = default; //
    virtual CalibrateCameraTaskResult calculate_calibration() = 0; //
  };

  std::unique_ptr<CalibrateCameraTaskInterface> make_calibrate_camera_task(
    const CalibrateContext &cal_cxt,
    const rclcpp::Time &now,
    std::unique_ptr<const CapturedImages> captured_images);
}
#endif //VLAM_CALIBRATE_HPP
