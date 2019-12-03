#ifndef FIDUCIAL_VLAM_FIDUCIAL_MATH_HPP
#define FIDUCIAL_VLAM_FIDUCIAL_MATH_HPP


#include <array>

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
// FiducialMath class
// ==============================================================================

  class FiducialMath
  {
    class CvFiducialMath;

    class SamFiducialMath;

    const bool sam_not_cv_;
    std::unique_ptr<CvFiducialMath> cv_;
    std::unique_ptr<SamFiducialMath> sam_;

  public:
    explicit FiducialMath(bool sam_not_cv,
                          double corner_measurement_sigma,
                          const CameraInfo &camera_info);

    explicit FiducialMath(bool sam_not_cv,
                          double corner_measurement_sigma,
                          const sensor_msgs::msg::CameraInfo &camera_info_msg);

    ~FiducialMath();

    TransformWithCovariance solve_t_camera_marker(const Observation &observation, double marker_length);

    TransformWithCovariance solve_t_map_camera(const Observations &observations,
                                               Map &map);

    Observations detect_markers(std::shared_ptr<cv_bridge::CvImage> &color,
                                std::shared_ptr<cv_bridge::CvImage> &color_marked);

    void annotate_image_with_marker_axis(std::shared_ptr<cv_bridge::CvImage> &color,
                                         const TransformWithCovariance &t_camera_marker);

    void update_map(const TransformWithCovariance &t_map_camera,
                    const Observations &observations,
                    Map &map);
  };
}

#endif //FIDUCIAL_VLAM_FIDUCIAL_MATH_HPP
