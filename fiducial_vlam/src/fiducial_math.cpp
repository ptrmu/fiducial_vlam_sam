
#include "fiducial_math.hpp"

#include "cv_bridge/cv_bridge.h"
#include "cv_utils.hpp"
#include "map.hpp"
#include "observation.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/opencv.hpp"
#include "ros2_shared/string_printf.hpp"
#include "tf_utils.hpp"
#include "transform_with_covariance.hpp"
#include "vloc_context.hpp"

namespace fiducial_vlam
{

// ==============================================================================
// CameraInfoImpl class
// ==============================================================================

  class CameraInfoImpl : public CameraInfoInterface
  {
    cv::Matx33d camera_matrix_;
    cv::Vec<double, 5> dist_coeffs_;

  public:
    explicit CameraInfoImpl(const sensor_msgs::msg::CameraInfo &msg) :
      camera_matrix_{3, 3, CV_64F, 0.}, dist_coeffs_{5, 1, CV_64F, 0.}
    {
      camera_matrix_(0, 0) = msg.k[0];
      camera_matrix_(0, 2) = msg.k[2];
      camera_matrix_(1, 1) = msg.k[4];
      camera_matrix_(1, 2) = msg.k[5];
      camera_matrix_(2, 2) = 1.;

      // ROS and OpenCV (and everybody?) agree on this ordering: k1, k2, t1 (p1), t2 (p2), k3
      if (!msg.d.empty()) {
        dist_coeffs_(0) = msg.d[0];
        dist_coeffs_(1) = msg.d[1];
        dist_coeffs_(2) = msg.d[2];
        dist_coeffs_(3) = msg.d[3];
        dist_coeffs_(4) = msg.d[4];
      }
    }

    const cv::Matx33d &camera_matrix() const override
    {
      return camera_matrix_;
    }

    const cv::Vec<double, 5> &dist_coeffs() const override
    {
      return dist_coeffs_;
    }
  };

  std::unique_ptr<const CameraInfoInterface> make_camera_info(const sensor_msgs::msg::CameraInfo &msg)
  {
    return std::make_unique<CameraInfoImpl>(msg);
  }

// ==============================================================================
// CvFiducialMathImpl class
// ==============================================================================

  class CvFiducialMathImpl : public CvFiducialMathInterface
  {
    const VlocContext &cxt_;
    SmoothObservationsInterface &so_;
    cv::Ptr<cv::aruco::Dictionary> localization_aruco_dictionary_;

  public:
    explicit CvFiducialMathImpl(const VlocContext &cxt,
                                SmoothObservationsInterface &so) :
      cxt_{cxt}, so_{so},
      localization_aruco_dictionary_{cv::aruco::getPredefinedDictionary(
        cv::aruco::PREDEFINED_DICTIONARY_NAME(cxt.loc_aruco_dictionary_id_))}
    {}

    Observations detect_markers(cv_bridge::CvImage &gray,
                                const rclcpp::Time &time_stamp,
                                cv::Mat &color_marked) override
    {
      auto detectorParameters = cv::aruco::DetectorParameters::create();
#if (CV_VERSION_MAJOR == 4)
//     0 = CORNER_REFINE_NONE,     ///< Tag and corners detection based on the ArUco approach
//     1 = CORNER_REFINE_SUBPIX,   ///< ArUco approach and refine the corners locations using corner subpixel accuracy
//     2 = CORNER_REFINE_CONTOUR,  ///< ArUco approach and refine the corners locations using the contour-points line fitting
//     3 = CORNER_REFINE_APRILTAG, ///< Tag and corners detection based on the AprilTag 2 approach @cite wang2016iros

      // Use the new AprilTag 2 corner algorithm, much better but much slower
      detectorParameters->cornerRefinementMethod = cxt_.loc_cv4_corner_refinement_method_;
#else
      // 0 = false
      // 1 = true
      detectorParameters->doCornerRefinement = cxt_.cv3_do_corner_refinement_;
#endif

      // Detect markers
      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f>> corners;
      cv::aruco::detectMarkers(gray.image, localization_aruco_dictionary_, corners, ids, detectorParameters);

      so_.smooth_observations(corners, ids, time_stamp);

      // Annotate the markers
      if (color_marked.dims != 0) {
        AnnotateImages::with_detected_markers(color_marked, corners, ids);
      }

      // return the corners as a list of observations
      return CvUtils::to_observations(ids, corners);
    }
  };

  std::unique_ptr<CvFiducialMathInterface> make_cv_fiducial_math(const VlocContext &cxt,
                                                                 SmoothObservationsInterface &so)
  {
    return std::make_unique<CvFiducialMathImpl>(cxt, so);
  }

// ==============================================================================
// CvLocalizeCameraImpl class
// ==============================================================================

  class CvLocalizeCameraImpl : public LocalizeCameraInterface
  {
    const VlocContext &cxt_;

  public:
    explicit CvLocalizeCameraImpl(const VlocContext &cxt) :
      cxt_{cxt}
    {}

    TransformWithCovariance solve_t_map_camera(const Observations &observations,
                                               const CameraInfoInterface &camera_info,
                                               const Map &map) override
    {
      // Build up two lists of corner points: 2D in the image frame, 3D in the marker frame
      std::vector<cv::Point3d> all_corners_f_map;
      std::vector<cv::Point2d> all_corners_f_image;

      for (const auto &observation : observations.observations()) {
        auto marker_ptr = map.find_marker_const(observation.id());
        if (marker_ptr != nullptr) {
          TFConvert::corners_f_map(marker_ptr->t_map_marker(), map.marker_length(), all_corners_f_map);
          observation.to_point_vector(all_corners_f_image);
        }
      }

      // If there are no known markers in the observation set, then don't
      // try to find the camera position
      if (all_corners_f_map.empty()) {
        return TransformWithCovariance{};
      }

      // Figure out camera location.
      cv::Vec3d rvec, tvec;
      try {
        cv::solvePnP(all_corners_f_map, all_corners_f_image,
                     camera_info.camera_matrix(), camera_info.dist_coeffs(),
                     rvec, tvec);

        // For certain cases, there is a chance that the multi marker solvePnP will
        // return the mirror of the correct solution. So try solvePn[Ransac as well.
        if (all_corners_f_image.size() > 1 * 4 && all_corners_f_image.size() < 4 * 4) {
          cv::Vec3d rvecRansac, tvecRansac;
          cv::solvePnPRansac(all_corners_f_map, all_corners_f_image,
                             camera_info.camera_matrix(), camera_info.dist_coeffs(),
                             rvecRansac, tvecRansac);

          // If the pose returned from the ransac version is very different from
          // that returned from the normal version, then use the ransac results.
          // solvePnp can sometimes pick up the wrong solution (a mirror solution).
          // solvePnpRansac does a better job in that case. But solvePnp does a
          // better job smoothing out image noise so it is prefered when it works.
          if (std::abs(rvec[0] - rvecRansac[0]) > 0.5 ||
              std::abs(rvec[1] - rvecRansac[1]) > 0.5 ||
              std::abs(rvec[2] - rvecRansac[2]) > 0.5) {
            rvec = rvecRansac;
            tvec = tvecRansac;
          }
        }
      } catch (cv::Exception &ex) {
        return TransformWithCovariance{};
      }

//      if (tvec[0] < 0) { // specific tests for bad pose determination
//        int xxx = 9;
//      }

      // rvec, tvec output from solvePnp "brings points from the model coordinate system to the
      // camera coordinate system". In this case the map frame is the model coordinate system.
      // So rvec, tvec are the transformation t_camera_map.
      auto tf2_t_map_camera = CvUtils::to_tf2_transform(rvec, tvec).inverse();
      return TransformWithCovariance(tf2_t_map_camera);
    }
  };

  std::unique_ptr<LocalizeCameraInterface> make_cv_localize_camera(const VlocContext &cxt)
  {
    return std::make_unique<CvLocalizeCameraImpl>(cxt);
  }

}
