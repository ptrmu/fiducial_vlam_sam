
#ifndef VLAM_CALIBRATE_HPP
#define VLAM_CALIBRATE_HPP

#include "ros2_shared/param_macros.hpp"
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
  PAMA_PARAM(cal_compute_on_thread, int, 1)                 /* Do heavy-duty computation on a thread. */\
  PAMA_PARAM(cal_calibration_style_to_save, int, 4)         /* The calibration style to save. */\
  PAMA_PARAM(cal_save_files_path, std::string, "cal_files") /* path to file to store calibration in. */\
  PAMA_PARAM(cal_camera_name, std::string, "Calibrated Camera") /* Name of the calibrated camera. */\
  PAMA_PARAM(cal_cmd, std::string, "")                      /* Calibrate command. */\
  PAMA_PARAM(cal_aruco_dictionary_id, int, 5)               /* dictionary id of aruco markers on the charuco calibration target  */ \
  PAMA_PARAM(cal_squares_x, int, 12)                        /* number of squares in the x direction on the charuco board  */ \
  PAMA_PARAM(cal_squares_y, int, 9)                         /* number of squares in the y direction on the charuco board  */ \
  PAMA_PARAM(cal_square_length, float, 0.060)               /* length of a square on the charuco board  */ \
  PAMA_PARAM(cal_upper_left_white_not_black, int, 0)        /* upper_left_white_not_black  */ \
  PAMA_PARAM(cal_marker_length, float, 0.0450)              /* length of a marker on the charuco board  */ \
  PAMA_PARAM(cal_stationary_capture_ms, int, 0)             /* Duration that calibration target needs to be stationary. 0 => infinite */ \
  PAMA_PARAM(cal_bootstrap_reserve_fraction, double, 0.2)   /* % of calibration images to reserve for bootstrap validation */ \
 /* End of list */

  struct CalibrateContext
  {
#undef PAMA_PARAM
#define PAMA_PARAM(n, t, d) PAMA_PARAM_DEFINE(n, t, d)
    PAMA_PARAMS_DEFINE(CAL_ALL_PARAMS)
  };

  std::string get_camera_info_file_name(const CalibrateContext &cal_cxt); //
  std::string get_captured_image_file_name(const CalibrateContext &cal_cxt); //
  std::string get_captured_image_file_name(const CalibrateContext &cal_cxt, std::uint32_t index); //
  std::string get_marked_image_file_name(const CalibrateContext &cal_cxt, std::uint32_t index); //
  std::string get_calibration_report_file_name(const CalibrateContext &cal_cxt); //

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
