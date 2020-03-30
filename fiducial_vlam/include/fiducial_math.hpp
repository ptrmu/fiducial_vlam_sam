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
  CXT_MACRO_MEMBER(       /* OpenCV 3.x argument to detect corners. 0 = false, 1 = true */ \
  cv3_do_corner_refinement, \
  int, 1) \
  CXT_MACRO_MEMBER(       /* OpenCV 4.x argument to detect corners. 0 = none, 1 = subpix, 2 = contour, 3 = apriltag */ \
  cv4_corner_refinement_method, \
  int, 2) \
  CXT_MACRO_MEMBER(       /* noise in detection of marker corners in the image (sigma in pixels) */ \
  corner_measurement_sigma, \
  double, 2.0) \
  CXT_MACRO_MEMBER(       /* use gtsam not opencv for localizing the camera */ \
  localize_camera_sam_not_cv, \
  int, 1) \
  CXT_MACRO_MEMBER(       /* use gtsam not opencv for building the marker map */ \
  build_marker_map_sam_not_cv, \
  int, 1) \
  CXT_MACRO_MEMBER(       /* Do heavy-duty computation on a thread. */ \
  compute_on_thread, \
  int, 1) \
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

    virtual TransformWithCovariance solve_t_camera_marker(const Observation &observation,
                                                          const CameraInfoInterface &camera_info,
                                                          double marker_length) = 0;

    virtual Observations detect_markers(cv_bridge::CvImage &gray,
                                        cv_bridge::CvImage &color_marked) = 0;

    virtual void annotate_image_with_marker_axis(cv_bridge::CvImage &color_marked,
                                                 const TransformWithCovariance &t_camera_marker,
                                                 const CameraInfoInterface &camera_info) = 0;
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
                                                                     CvFiducialMathInterface &fm,
                                                                     const Map &empty_map);

// ==============================================================================
// ProcessImageInterface class
// ==============================================================================

  class ProcessImageInterface
  {
  public:
    virtual ~ProcessImageInterface() = default;

    virtual Observations process_image(std::shared_ptr<cv_bridge::CvImage> &gray,
                                       cv_bridge::CvImage &color_marked) = 0;

    virtual TransformWithCovariance solve_t_map_camera(const Observations &observations,
                                                       const CameraInfoInterface &camera_info,
                                                       const Map &map) = 0;
  };

}

#endif //FIDUCIAL_VLAM_FIDUCIAL_MATH_HPP
