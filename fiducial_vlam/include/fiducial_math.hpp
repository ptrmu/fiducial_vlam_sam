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

    int &sam_not_cv_;
    int &sfm_not_slam_;
    std::unique_ptr<CvFiducialMath> cv_;
    std::unique_ptr<SamFiducialMath> sam_;

  public:
    explicit FiducialMath(int &sam_not_cv, int &sfm_not_slam,
                          double &corner_measurement_sigma);

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
  };
}

#endif //FIDUCIAL_VLAM_FIDUCIAL_MATH_HPP
