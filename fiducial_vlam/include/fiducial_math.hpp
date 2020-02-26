#ifndef FIDUCIAL_VLAM_FIDUCIAL_MATH_HPP
#define FIDUCIAL_VLAM_FIDUCIAL_MATH_HPP


#include <array>

#include "ros2_shared/context_macros.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace cv_bridge
{
  class CvImage;
}

namespace gtsam
{
  class Cal3DS2;
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

    const gtsam::Cal3DS2 &cal3ds2() const;

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
  CXT_MACRO_MEMBER(       /* When making maps used the SlamTask. */ \
  use_slam_task, \
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
// UpdateMapInterface class
// ==============================================================================

  class UpdateMapInterface
  {
  public:
    virtual ~UpdateMapInterface() = default;

    virtual void update_map(const Observations &observations,
                            const CameraInfo &camera_info,
                            Map &map) = 0;

    virtual void update_map_for_publishing(Map &map) = 0;

    virtual std::string update_map_cmd(std::string &cmd) = 0;
  };

// ==============================================================================
// FiducialMath class
// ==============================================================================

  class CvFiducialMath;

  class SamFiducialMath;

  class FiducialMath
  {
    std::unique_ptr<CvFiducialMath> cv_;
    std::unique_ptr<SamFiducialMath> sam_;
    std::unique_ptr<UpdateMapInterface> update_;

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
//
//    void update_map(const Observations &observations,
//                    const CameraInfo &camera_info,
//                    Map &map);
//
//    void update_map_for_publishing(Map &map);
//
//    std::string update_map_cmd(std::string &cmd, const Map &empty_map_);
  };

// ==============================================================================
// CvFiducialMathInterface class
// ==============================================================================

  class CvFiducialMathInterface
  {
  public:
    virtual ~CvFiducialMathInterface() = default;

    virtual TransformWithCovariance solve_t_camera_marker(const Observation &observation,
                                                          const CameraInfo &camera_info,
                                                          double marker_length) = 0;

    virtual Observations detect_markers(std::shared_ptr<cv_bridge::CvImage> &image,
                                        std::shared_ptr<cv_bridge::CvImage> &color_marked) = 0;

    virtual void annotate_image_with_marker_axis(std::shared_ptr<cv_bridge::CvImage> &color_marked,
                                                 const TransformWithCovariance &t_camera_marker,
                                                 const CameraInfo &camera_info) = 0;
  };

// ==============================================================================
// LocalizeCameraInterface class
// ==============================================================================

  class LocalizeCameraInterface
  {
  public:
    virtual ~LocalizeCameraInterface() = default;

    virtual TransformWithCovariance solve_t_map_camera(const Observations &observations,
                                                       const CameraInfo &camera_info,
                                                       Map &map) = 0;
  };

// ==============================================================================
// BuildMarkerMapInterface class
// ==============================================================================

  class BuildMarkerMapInterface
  {
  public:
    virtual ~BuildMarkerMapInterface() = default;

    virtual void process_observations(const Observations &observations,
                                      const CameraInfo &camera_info) = 0;

    virtual std::string update_map(Map &map) = 0;

    virtual std::string build_marker_map_cmd(std::string &cmd) = 0;
  };


// ==============================================================================
// factories
// ==============================================================================

//  std::unique_ptr<UpdateMapInterface> slam_task_factory(FiducialMath &fm,
//                                                        const FiducialMathContext &cxt,
//                                                        const Map &empty_map);

  std::unique_ptr<CvFiducialMathInterface> cv_fiducial_math_factory(const FiducialMathContext &cxt);

  std::unique_ptr<BuildMarkerMapInterface> sam_build_marker_map_factory(CvFiducialMathInterface &fm,
                                                                        const FiducialMathContext &cxt,
                                                                        const Map &empty_map);

//  std::unique_ptr<BuildMarkerMapInterface> cv_build_marker_map_factory(CvFiducialMathInterface &fm,
//                                                                       const FiducialMathContext &cxt,
//                                                                       const Map &empty_map);

}

#endif //FIDUCIAL_VLAM_FIDUCIAL_MATH_HPP
