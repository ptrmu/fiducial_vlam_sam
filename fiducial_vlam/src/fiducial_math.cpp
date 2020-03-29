
#include "fiducial_math.hpp"

#include "map.hpp"
#include "observation.hpp"
#include "ros2_shared/string_printf.hpp"
#include "transform_with_covariance.hpp"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/aruco.hpp"
#include "opencv2/calib3d/calib3d.hpp"

//#include <gtsam/geometry/Cal3DS2.h>
//#include <gtsam/geometry/PinholeCamera.h>
//#include <gtsam/geometry/Point3.h>
//#include <gtsam/geometry/Pose3.h>
//#include "gtsam/inference/Symbol.h"
//#include <gtsam/nonlinear/ISAM2.h>
//#include <gtsam/nonlinear/NonlinearFactorGraph.h>
//#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
//#include <gtsam/nonlinear/Marginals.h>
//#include <gtsam/slam/BetweenFactor.h>
//#include <gtsam/slam/PriorFactor.h>
//#include <gtsam/slam/ProjectionFactor.h>

namespace fiducial_vlam
{
// ==============================================================================
// Convert class
// ==============================================================================

  struct Convert
  {
    template<class TPoint>
    static std::vector<TPoint> corners_f_marker(double marker_length)
    {
      return std::vector<TPoint>{
        TPoint{-marker_length / 2.0, marker_length / 2.0, 0.0},
        TPoint{marker_length / 2.0, marker_length / 2.0, 0.0},
        TPoint{marker_length / 2.0, -marker_length / 2.0, 0.0},
        TPoint{-marker_length / 2.0, -marker_length / 2.0, 0.0}};
    }

    template<class TPoint>
    static void
    corners_f_map(const TransformWithCovariance &t_map_marker, double marker_length, std::vector<TPoint> &destination)
    {
      auto &t{t_map_marker.transform()};
      destination.insert(destination.end(), {
        to_point<TPoint>(t * tf2::Vector3{-marker_length / 2.0, marker_length / 2.0, 0.0}),
        to_point<TPoint>(t * tf2::Vector3{marker_length / 2.0, marker_length / 2.0, 0.0}),
        to_point<TPoint>(t * tf2::Vector3{marker_length / 2.0, -marker_length / 2.0, 0.0}),
        to_point<TPoint>(t * tf2::Vector3{-marker_length / 2.0, -marker_length / 2.0, 0.0})});
    }

    template<class TPoint>
    static TPoint to_point(const Vector3WithCovariance &v3wc)
    {
      return TPoint{v3wc.vector3().x(),
                    v3wc.vector3().y(),
                    v3wc.vector3().z()};
    }

    template<class TPoint>
    static TPoint to_point(const tf2::Vector3 &v3)
    {
      return TPoint{v3.x(),
                    v3.y(),
                    v3.z()};
    }

    template<class TPoint>
    static std::vector<TPoint> to_corner_points(const std::vector<Vector3WithCovariance> &corners)
    {
      std::vector<TPoint> corner_points;
      for (auto &corner : corners) {
        corner_points.emplace_back(to_point<TPoint>(corner));
      }
      return corner_points;
    }
  };

// ==============================================================================
// CvUtil class
// ==============================================================================

  struct CvUtil
  {
    static tf2::Transform to_tf2_transform(const cv::Vec3d &rvec, const cv::Vec3d &tvec)
    {
      cv::Mat rmat;
      cv::Rodrigues(rvec, rmat);
      tf2::Matrix3x3 m;
      for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 3; col++) {
          m[row][col] = rmat.at<double>(row, col);  // Row- vs. column-major order
        }
      }
      return tf2::Transform(m, tf2::Vector3{tvec[0], tvec[1], tvec[2]});
    }

    static void to_cv_rvec_tvec(const TransformWithCovariance &t, cv::Vec3d &rvec, cv::Vec3d &tvec)
    {
      auto c = t.transform().getOrigin();
      tvec[0] = c.x();
      tvec[1] = c.y();
      tvec[2] = c.z();
      auto R = t.transform().getBasis();
      cv::Mat rmat(3, 3, CV_64FC1);
      for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 3; col++) {
          rmat.at<double>(row, col) = R[row][col];
        }
      }
      cv::Rodrigues(rmat, rvec);
    }

    static Observations to_observations(const std::vector<int> &ids,
                                        const std::vector<std::vector<cv::Point2f>> &corners)
    {
      Observations observations;
      for (int i = 0; i < ids.size(); i += 1) {
        observations.add(Observation(ids[i],
                                     corners[i][0].x, corners[i][0].y,
                                     corners[i][1].x, corners[i][1].y,
                                     corners[i][2].x, corners[i][2].y,
                                     corners[i][3].x, corners[i][3].y));
      }
      return observations;
    }
  };

// ==============================================================================
// CameraInfoImpl class
// ==============================================================================

  class CameraInfoImpl : public CameraInfoInterface
  {
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

  public:
    CameraInfoImpl(const sensor_msgs::msg::CameraInfo &msg) :
      camera_matrix_{3, 3, CV_64F, 0.}, dist_coeffs_{5, 1, CV_64F, 0.}
    {
      camera_matrix_.at<double>(0, 0) = msg.k[0];
      camera_matrix_.at<double>(0, 2) = msg.k[2];
      camera_matrix_.at<double>(1, 1) = msg.k[4];
      camera_matrix_.at<double>(1, 2) = msg.k[5];
      camera_matrix_.at<double>(2, 2) = 1.;

      // ROS and OpenCV (and everybody?) agree on this ordering: k1, k2, t1 (p1), t2 (p2), k3
      dist_coeffs_.at<double>(0) = msg.d[0];
      dist_coeffs_.at<double>(1) = msg.d[1];
      dist_coeffs_.at<double>(2) = msg.d[2];
      dist_coeffs_.at<double>(3) = msg.d[3];
      dist_coeffs_.at<double>(4) = msg.d[4];
    }

    const cv::Mat &camera_matrix() const override
    {
      return camera_matrix_;
    }

    const cv::Mat &dist_coeffs() const override
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
    const FiducialMathContext &cxt_;

    static void drawDetectedMarkers(cv::InputOutputArray image,
                                    cv::InputArrayOfArrays corners,
                                    cv::InputArray ids)
    {
      // calculate colors
      auto borderColor = cv::Scalar(0, 255, 0);
      cv::Scalar textColor = borderColor;
      cv::Scalar cornerColor = borderColor;

      std::swap(textColor.val[0], textColor.val[1]);     // text color just sawp G and R
      std::swap(cornerColor.val[1], cornerColor.val[2]); // corner color just sawp G and B

      int nMarkers = static_cast<int>(corners.total());
      for (int i = 0; i < nMarkers; i++) {

        cv::Mat currentMarker = corners.getMat(i);
        CV_Assert((currentMarker.total() == 4) && (currentMarker.type() == CV_32FC2));

        // draw marker sides
        for (int j = 0; j < 4; j++) {
          cv::Point2f p0, p1;
          p0 = currentMarker.ptr<cv::Point2f>(0)[j];
          p1 = currentMarker.ptr<cv::Point2f>(0)[(j + 1) % 4];
          line(image, p0, p1, borderColor, 1);
        }

        // draw first corner mark
        rectangle(image,
                  currentMarker.ptr<cv::Point2f>(0)[0] - cv::Point2f(3, 3),
                  currentMarker.ptr<cv::Point2f>(0)[0] + cv::Point2f(3, 3),
                  cornerColor, 1, cv::LINE_AA);

        // draw ID
//      if (ids.total() != 0) {
//        cv::Point2f cent(0, 0);
//        for (int p = 0; p < 4; p++)
//          cent += currentMarker.ptr<cv::Point2f>(0)[p];
//
//        cent = cent / 4.;
//        std::stringstream s;
//        s << "id=" << ids.getMat().ptr<int>(0)[i];
//        putText(image, s.str(), cent, cv::FONT_HERSHEY_SIMPLEX, 0.5, textColor, 2);
//      }
      }
    }

  public:
    explicit CvFiducialMathImpl(const FiducialMathContext &cxt) :
      cxt_{cxt}
    {}

    TransformWithCovariance solve_t_camera_marker(const Observation &observation,
                                                  const CameraInfoInterface &camera_info,
                                                  double marker_length) override
    {
      // Build up two lists of corner points: 2D in the image frame, 3D in the marker frame
      auto corners_f_marker{Convert::corners_f_marker<cv::Point3d>(marker_length)};
      auto corners_f_image{observation.to_point_vector<cv::Point2d>()};

      // Figure out marker pose.
      cv::Vec3d rvec, tvec;
      cv::solvePnP(corners_f_marker, corners_f_image,
                   camera_info.camera_matrix(), camera_info.dist_coeffs(),
                   rvec, tvec);

      // rvec, tvec output from solvePnp "brings points from the model coordinate system to the
      // camera coordinate system". In this case the marker frame is the model coordinate system.
      // So rvec, tvec are the transformation t_camera_marker.
      return TransformWithCovariance(CvUtil::to_tf2_transform(rvec, tvec));
    }

    Observations detect_markers(std::shared_ptr<cv_bridge::CvImage> &gray,
                                std::shared_ptr<cv_bridge::CvImage> &color_marked) override
    {
      // Todo: make the dictionary a parameter
      auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
      auto detectorParameters = cv::aruco::DetectorParameters::create();
#if (CV_VERSION_MAJOR == 4)
//     0 = CORNER_REFINE_NONE,     ///< Tag and corners detection based on the ArUco approach
//     1 = CORNER_REFINE_SUBPIX,   ///< ArUco approach and refine the corners locations using corner subpixel accuracy
//     2 = CORNER_REFINE_CONTOUR,  ///< ArUco approach and refine the corners locations using the contour-points line fitting
//     3 = CORNER_REFINE_APRILTAG, ///< Tag and corners detection based on the AprilTag 2 approach @cite wang2016iros

      // Use the new AprilTag 2 corner algorithm, much better but much slower
      detectorParameters->cornerRefinementMethod = cxt_.cv4_corner_refinement_method_;
#else
      // 0 = false
      // 1 = true
      detectorParameters->doCornerRefinement = cxt_.cv3_do_corner_refinement_;
#endif

      // Detect markers
      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f>> corners;
      cv::aruco::detectMarkers(gray->image, dictionary, corners, ids, detectorParameters);

      // Annotate the markers
      if (color_marked) {
        drawDetectedMarkers(color_marked->image, corners, ids);
      }

      // return the corners as a list of observations
      return CvUtil::to_observations(ids, corners);

    }

    void annotate_image_with_marker_axis(std::shared_ptr<cv_bridge::CvImage> &color_marked,
                                         const TransformWithCovariance &t_camera_marker,
                                         const CameraInfoInterface &camera_info) override
    {
      cv::Vec3d rvec, tvec;
      CvUtil::to_cv_rvec_tvec(t_camera_marker, rvec, tvec);

      cv::aruco::drawAxis(color_marked->image,
                          camera_info.camera_matrix(), camera_info.dist_coeffs(),
                          rvec, tvec, 0.1);
    }
  };

  std::unique_ptr<CvFiducialMathInterface> make_cv_fiducial_math(const FiducialMathContext &cxt)
  {
    return std::make_unique<CvFiducialMathImpl>(cxt);
  }

// ==============================================================================
// CvLocalizeCameraImpl class
// ==============================================================================

  class CvLocalizeCameraImpl : public LocalizeCameraInterface
  {
    const FiducialMathContext &cxt_;

  public:
    explicit CvLocalizeCameraImpl(const FiducialMathContext &cxt) :
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
          Convert::corners_f_map(marker_ptr->t_map_marker(), map.marker_length(), all_corners_f_map);
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

      if (tvec[0] < 0) { // specific tests for bad pose determination
        int xxx = 9;
      }

      // rvec, tvec output from solvePnp "brings points from the model coordinate system to the
      // camera coordinate system". In this case the map frame is the model coordinate system.
      // So rvec, tvec are the transformation t_camera_map.
      auto tf2_t_map_camera = CvUtil::to_tf2_transform(rvec, tvec).inverse();
      return TransformWithCovariance(tf2_t_map_camera);
    }
  };

  std::unique_ptr<LocalizeCameraInterface> make_cv_localize_camera(const FiducialMathContext &cxt)
  {
    return std::make_unique<CvLocalizeCameraImpl>(cxt);
  }

}
