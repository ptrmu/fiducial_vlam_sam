#ifndef FIDUCIAL_VLAM_FIDUCIAL_MATH_HPP
#define FIDUCIAL_VLAM_FIDUCIAL_MATH_HPP


#include <array>

#include "ros2_shared/context_macros.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace cv_bridge
{
  class CvImage;
}

//namespace gtsam
//{
//  class Cal3DS2;
//}

namespace cv
{
  class Mat;
}

namespace fiducial_vlam
{
  class Observation;

  class Observations;

  class TransformWithCovariance;

  class Map;

// ==============================================================================
// CameraInfoInterface class
// ==============================================================================

  class CameraInfoInterface
  {
  public:
    virtual ~CameraInfoInterface() = default;

    virtual const cv::Mat &camera_matrix() const = 0;

    virtual const cv::Mat &dist_coeffs() const = 0;
  };

  std::unique_ptr<const CameraInfoInterface> make_camera_info(const sensor_msgs::msg::CameraInfo &msg);

#if 0
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

      cv::Mat &camera_matrix();

  //
  //    bool is_valid() const
  //    { return cv_ != nullptr; }
    };
#endif

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

#if 0
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
    };
#endif

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

    virtual Observations detect_markers(std::shared_ptr<cv_bridge::CvImage> &image,
                                        std::shared_ptr<cv_bridge::CvImage> &color_marked) = 0;

    virtual void annotate_image_with_marker_axis(std::shared_ptr<cv_bridge::CvImage> &color_marked,
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

}

#endif //FIDUCIAL_VLAM_FIDUCIAL_MATH_HPP
