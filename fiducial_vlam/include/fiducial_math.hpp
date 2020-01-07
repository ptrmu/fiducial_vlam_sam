#ifndef FIDUCIAL_VLAM_FIDUCIAL_MATH_HPP
#define FIDUCIAL_VLAM_FIDUCIAL_MATH_HPP


#include <array>

#include "ros2_shared/context_macros.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace cv_bridge
{
  class CvImage;
}

namespace fiducial_vlam
{
  class Observation;

  class Observations;

  class TransformWithCovariance;

  class Map;

// ==============================================================================
// CameraInfo class
// ==============================================================================

  class CameraInfo
  {
    class CvCameraInfo;

    std::shared_ptr<CvCameraInfo> cv_;

  public:
    CameraInfo();

    explicit CameraInfo(const sensor_msgs::msg::CameraInfo &camera_info);

    auto &cv() const
    { return cv_; }

    bool is_valid() const
    { return cv_ != nullptr; }
  };

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
  double, 1.0) \
  CXT_MACRO_MEMBER(       /* use gtsam for fiducial calculations not opencv */ \
  sam_not_cv, \
  int, 1) \
  CXT_MACRO_MEMBER(       /* use gtsam in Structure From Motion rather than Simultaneous Localization And Mapping mode */ \
  sfm_not_slam, \
  int, 0) \
  CXT_MACRO_MEMBER(       /* When making maps, optimize the measurements from multiple frames. */ \
  multi_frame_optimization, \
  int, 1) \
  CXT_MACRO_MEMBER(       /* When making maps with multiple frames, use ISAM2. */ \
  use_isam, \
  int, 1) \
  /* End of list */

  struct FiducialMathContext
  {
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)
    FM_ALL_PARAMS
  };

// ==============================================================================
// FiducialMath class
// ==============================================================================

  class FiducialMath
  {
    class CvFiducialMath;

    class SamFiducialMath;

    class UpdateFiducialMath;

    std::unique_ptr<CvFiducialMath> cv_;
    std::unique_ptr<SamFiducialMath> sam_;
    std::unique_ptr<UpdateFiducialMath> update_;

  public:
    explicit FiducialMath(FiducialMathContext &cxt);

    ~FiducialMath();

    TransformWithCovariance solve_t_camera_marker(const Observation &observation,
                                                  const CameraInfo &camera_info,
                                                  double marker_length);

    TransformWithCovariance solve_t_map_camera(const Observations &observations,
                                               const CameraInfo &camera_info,
                                               Map &map);

    Observations detect_markers(std::shared_ptr<cv_bridge::CvImage> &color,
                                std::shared_ptr<cv_bridge::CvImage> &color_marked);

    void annotate_image_with_marker_axis(std::shared_ptr<cv_bridge::CvImage> &color,
                                         const TransformWithCovariance &t_camera_marker,
                                         const CameraInfo &camera_info);

    void update_map(const Observations &observations,
                    const CameraInfo &camera_info,
                    Map &map);

    void update_map_for_publishing(Map &map);

    std::string update_map_cmd(std::string &cmd);
  };
}

#endif //FIDUCIAL_VLAM_FIDUCIAL_MATH_HPP
