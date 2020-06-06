#ifndef FIDUCIAL_VLAM_FIDUCIAL_MATH_HPP
#define FIDUCIAL_VLAM_FIDUCIAL_MATH_HPP

#include "opencv2/core/types.hpp"
#include "ros2_shared/context_macros.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace cv_bridge
{
  class CvImage;
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
  class VlocContext; //
  class SmoothObservationsInterface; //

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
  CXT_MACRO_MEMBER(corner_measurement_sigma, double, 2.0)     /* Noise model in GTSAM for marker corners in the image (sigma in pixels) */\
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
                                        const rclcpp::Time &time_stamp,
                                        cv::Mat &color_marked) = 0;
  };

  std::unique_ptr<CvFiducialMathInterface> make_cv_fiducial_math(const VlocContext &cxt,
                                                                 SmoothObservationsInterface &so);

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

  std::unique_ptr<LocalizeCameraInterface> make_cv_localize_camera(const VlocContext &cxt);

  std::unique_ptr<LocalizeCameraInterface> make_sam_localize_camera(const VlocContext &cxt,
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

    virtual std::string map_cmd(std::string &cmd) = 0;
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

    virtual void smooth_observations(std::vector<std::vector<cv::Point2f>> &aruco_corners,
                                     std::vector<int> &aruco_ids,
                                     const rclcpp::Time &time_stamp) = 0;
  };

  std::unique_ptr<SmoothObservationsInterface> make_smooth_observations(const VlocContext &cxt);
}

#endif //FIDUCIAL_VLAM_FIDUCIAL_MATH_HPP
