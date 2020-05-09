#ifndef FIDUCIAL_VLAM_FIDUCIAL_MATH_HPP
#define FIDUCIAL_VLAM_FIDUCIAL_MATH_HPP


#include <array>

#include "ros2_shared/context_macros.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace cv_bridge
{
  class CvImage;
}

namespace cv
{
  class Mat;
}

namespace rclcpp
{
  class Time;
}

namespace fiducial_vlam
{
  class CalibrateContext; //
  class Observation; //
  class Observations; //
  class TransformWithCovariance; //
  class Map; //

// ==============================================================================
// CameraInfoInterface class
// ==============================================================================

  class CameraInfoInterface
  {
  public:
    virtual ~CameraInfoInterface() = default; //
    virtual const cv::Mat &camera_matrix() const = 0; //
    virtual const cv::Mat &dist_coeffs() const = 0; //
  };

  std::unique_ptr<const CameraInfoInterface> make_camera_info(const sensor_msgs::msg::CameraInfo &msg);

// ==============================================================================
// FiducialMathContext class
// ==============================================================================

#define FM_ALL_PARAMS \
  CXT_MACRO_MEMBER(cv3_do_corner_refinement, int, 1)          /* OpenCV 3.x argument to detect corners. 0 = false, 1 = true */\
  CXT_MACRO_MEMBER(cv4_corner_refinement_method, int, 2)      /* OpenCV 4.x argument to detect corners. 0 = none, 1 = subpix, 2 = contour, 3 = apriltag */\
  CXT_MACRO_MEMBER(localization_aruco_dictionary_id, int, 10) /* aruco dictionary id for localization markers  */ \
  CXT_MACRO_MEMBER(corner_measurement_sigma, double, 2.0)     /* noise in detection of marker corners in the image (sigma in pixels) */\
  CXT_MACRO_MEMBER(localize_camera_sam_not_cv, int, 1)        /* use gtsam not opencv for localizing the camera */\
  CXT_MACRO_MEMBER(build_marker_map_sam_not_cv, int, 1)       /* use gtsam not opencv for building the marker map */\
  CXT_MACRO_MEMBER(compute_on_thread, int, 1)                 /* Do heavy-duty computation on a thread. */\
  /* End of list */

  struct FiducialMathContext
  {
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)
    FM_ALL_PARAMS
  };

// ==============================================================================
// CvFiducialMathInterface class
// ==============================================================================

  class CvFiducialMathInterface
  {
  public:
    virtual ~CvFiducialMathInterface() = default;

    virtual Observations detect_markers(cv_bridge::CvImage &gray,
                                        cv::Mat &color_marked) = 0;
  };

  std::unique_ptr<CvFiducialMathInterface> make_cv_fiducial_math(const FiducialMathContext &cxt);

// ==============================================================================
// LocalizeCameraInterface class
// ==============================================================================

  class LocalizeCameraInterface
  {
  public:
    virtual ~LocalizeCameraInterface() = default;

    virtual TransformWithCovariance solve_t_map_camera(const Observations &observations,
                                                       const CameraInfoInterface &camera_info,
                                                       const Map &map) = 0;
  };

  std::unique_ptr<LocalizeCameraInterface> make_cv_localize_camera(const FiducialMathContext &cxt);

  std::unique_ptr<LocalizeCameraInterface> make_sam_localize_camera(const FiducialMathContext &cxt,
                                                                    LocalizeCameraInterface &cv_lc);

// ==============================================================================
// BuildMarkerMapInterface class
// ==============================================================================

  class BuildMarkerMapInterface
  {
  public:
    virtual ~BuildMarkerMapInterface() = default;

    virtual void process_observations(std::unique_ptr<const Observations> observations,
                                      std::unique_ptr<const CameraInfoInterface> camera_info) = 0;

    virtual std::string build_marker_map(Map &map) = 0;

    virtual std::string build_marker_map_cmd(std::string &cmd) = 0;
  };

  std::unique_ptr<BuildMarkerMapInterface> make_sam_build_marker_map(const FiducialMathContext &cxt,
                                                                     const Map &empty_map);

// ==============================================================================
// ProcessImageInterface class
// ==============================================================================

  class ProcessImageInterface
  {
  public:
    virtual ~ProcessImageInterface() = default;

    virtual Observations process_image(std::shared_ptr<cv_bridge::CvImage> &gray,
                                       const rclcpp::Time &time_stamp,
                                       cv::Mat &color_marked) = 0;

    virtual TransformWithCovariance solve_t_map_camera(const Observations &observations,
                                                       const CameraInfoInterface &camera_info,
                                                       const Map &map) = 0;
  };

// ==============================================================================
// SmoothObservationsInterface class
// ==============================================================================

  class SmoothObservationsInterface
  {
  public:
    virtual ~SmoothObservationsInterface() = default;

    virtual void smooth_observations(Observations &observations) = 0;
  };

  std::unique_ptr<SmoothObservationsInterface> make_smooth_observations(const FiducialMathContext &cxt);
}

#endif //FIDUCIAL_VLAM_FIDUCIAL_MATH_HPP
