
#include "fiducial_math.hpp"

#include "map.hpp"
#include "observation.hpp"
#include "ros2_shared/string_printf.hpp"
#include "transform_with_covariance.hpp"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/aruco.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include "gtsam/inference/Symbol.h"
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>

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
        TPoint{-marker_length / 2.f, marker_length / 2.f, 0.f},
        TPoint{marker_length / 2.f, marker_length / 2.f, 0.f},
        TPoint{marker_length / 2.f, -marker_length / 2.f, 0.f},
        TPoint{-marker_length / 2.f, -marker_length / 2.f, 0.f}};
    }

    template<class TPoint>
    static TPoint to_point(const Vector3WithCovariance &v3wc)
    {
      return TPoint{v3wc.vector3().x(),
                    v3wc.vector3().y(),
                    v3wc.vector3().z()};
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
// CameraInfo::CvCameraInfo class
// ==============================================================================

  class CameraInfo::CvCameraInfo
  {
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    boost::shared_ptr<gtsam::Cal3DS2> cal3ds2_;

  public:
    CvCameraInfo() = delete;

    // C++ mystery: in constructor below
    //  dist_coeffs_(5, 1, CV_64F) creates a 5x1 matrix
    //  dist_coeffs_{5, 1, CV_64F, 0.} creates a 5x1 matrix
    // but
    //  dist_coeffs_{5, 1, CV_64F} creates a 3x1 matrix
    // Someday should figure this out!
    explicit CvCameraInfo(const sensor_msgs::msg::CameraInfo &msg) :
      camera_matrix_{3, 3, CV_64F, 0.}, dist_coeffs_{5, 1, CV_64F, 0.},
      cal3ds2_{boost::shared_ptr<gtsam::Cal3DS2>{new gtsam::Cal3DS2{
        msg.k[0],  // fx
        msg.k[4],  // fy
        1.0, // s
        msg.k[2],  // u0
        msg.k[5],  // v0
        msg.d[0], // k1
        msg.d[1], // k2
        msg.d[2], // p1
        msg.d[3]}}}// p2
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

    auto &camera_matrix()
    { return camera_matrix_; }

    auto &dist_coeffs()
    { return dist_coeffs_; }

    auto &cal3ds2()
    { return cal3ds2_; }
  };

// ==============================================================================
// CameraInfo class
// ==============================================================================

  CameraInfo::CameraInfo() = default;

  CameraInfo::CameraInfo(const sensor_msgs::msg::CameraInfo &camera_info_msg)
    : cv_(std::make_shared<CameraInfo::CvCameraInfo>(camera_info_msg))
  {}

// ==============================================================================
// drawDetectedMarkers function
// ==============================================================================

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

// ==============================================================================
// FiducialMath::CvFiducialMath class
// ==============================================================================

  class FiducialMath::CvFiducialMath
  {
  public:
    FiducialMathContext &cxt_;

    CvFiducialMath(FiducialMathContext &cxt) :
      cxt_{cxt}
    {}

    TransformWithCovariance solve_t_camera_marker(
      const Observation &observation,
      const CameraInfo &camera_info,
      double marker_length)
    {
      // Build up two lists of corner points: 2D in the image frame, 3D in the marker frame
      std::vector<cv::Point3d> corners_f_marker;
      std::vector<cv::Point2f> corners_f_image;

      append_corners_f_marker(marker_length, corners_f_marker);
      append_corners_f_image(observation, corners_f_image);

      // Figure out image location.
      cv::Vec3d rvec, tvec;
      cv::solvePnP(corners_f_marker, corners_f_image,
                   camera_info.cv()->camera_matrix(), camera_info.cv()->dist_coeffs(),
                   rvec, tvec);

      // rvec, tvec output from solvePnp "brings points from the model coordinate system to the
      // camera coordinate system". In this case the marker frame is the model coordinate system.
      // So rvec, tvec are the transformation t_camera_marker.
      return TransformWithCovariance(to_tf2_transform(rvec, tvec));
    }

    TransformWithCovariance solve_t_map_camera(const Observations &observations,
                                               const CameraInfo &camera_info,
                                               Map &map)
    {
      auto t_map_markers = map.find_t_map_markers(observations);

      // Build up two lists of corner points: 2D in the image frame, 3D in the marker frame
      std::vector<cv::Point3d> all_corners_f_map;
      std::vector<cv::Point2f> all_corners_f_image;

      for (int i = 0; i < observations.size(); i += 1) {
        auto &observation = observations.observations()[i];
        auto &t_map_marker = t_map_markers[i];
        if (t_map_marker.is_valid()) {
          append_corners_f_map(t_map_marker, map.marker_length(), all_corners_f_map);
          append_corners_f_image(observation, all_corners_f_image);
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
                     camera_info.cv()->camera_matrix(), camera_info.cv()->dist_coeffs(),
                     rvec, tvec);

        // For certain cases, there is a chance that the multi marker solvePnP will
        // return the mirror of the correct solution. So try solvePn[Ransac as well.
        if (all_corners_f_image.size() > 1 * 4 && all_corners_f_image.size() < 4 * 4) {
          cv::Vec3d rvecRansac, tvecRansac;
          cv::solvePnPRansac(all_corners_f_map, all_corners_f_image,
                             camera_info.cv()->camera_matrix(), camera_info.cv()->dist_coeffs(),
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
      auto tf2_t_map_camera = to_tf2_transform(rvec, tvec).inverse();
      return TransformWithCovariance(tf2_t_map_camera);
    }

    Observations detect_markers(cv_bridge::CvImagePtr &gray,
                                std::shared_ptr<cv_bridge::CvImage> &color_marked)
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
      return to_observations(ids, corners);
    }

    void annotate_image_with_marker_axis(std::shared_ptr<cv_bridge::CvImage> &color_marked,
                                         const TransformWithCovariance &t_camera_marker,
                                         const CameraInfo &camera_info)
    {
      cv::Vec3d rvec;
      cv::Vec3d tvec;
      to_cv_rvec_tvec(t_camera_marker, rvec, tvec);

      cv::aruco::drawAxis(color_marked->image,
                          camera_info.cv()->camera_matrix(), camera_info.cv()->dist_coeffs(),
                          rvec, tvec, 0.1);
    }

    void update_marker_simple_average(Marker &existing, const TransformWithCovariance &another_twc)
    {
      if (!existing.is_fixed()) {
        auto t_map_marker = existing.t_map_marker();  // Make a copy
        auto update_count = existing.update_count();
        t_map_marker.update_simple_average(another_twc, update_count);
        existing.set_t_map_marker(t_map_marker);
        existing.set_update_count(update_count + 1);
      }
    }

    void append_corners_f_map(const TransformWithCovariance &t_map_marker,
                              double marker_length,
                              std::vector<cv::Point3d> &corners_f_map)
    {
      // Build up a list of the corner locations in the marker frame.
      tf2::Vector3 corner0_f_marker(-marker_length / 2.f, marker_length / 2.f, 0.f);
      tf2::Vector3 corner1_f_marker(marker_length / 2.f, marker_length / 2.f, 0.f);
      tf2::Vector3 corner2_f_marker(marker_length / 2.f, -marker_length / 2.f, 0.f);
      tf2::Vector3 corner3_f_marker(-marker_length / 2.f, -marker_length / 2.f, 0.f);

      // Transform the corners to the map frame.
      const auto &t_map_marker_tf = t_map_marker.transform();
      auto corner0_f_map = t_map_marker_tf * corner0_f_marker;
      auto corner1_f_map = t_map_marker_tf * corner1_f_marker;
      auto corner2_f_map = t_map_marker_tf * corner2_f_marker;
      auto corner3_f_map = t_map_marker_tf * corner3_f_marker;

      corners_f_map.emplace_back(cv::Point3d(corner0_f_map.x(), corner0_f_map.y(), corner0_f_map.z()));
      corners_f_map.emplace_back(cv::Point3d(corner1_f_map.x(), corner1_f_map.y(), corner1_f_map.z()));
      corners_f_map.emplace_back(cv::Point3d(corner2_f_map.x(), corner2_f_map.y(), corner2_f_map.z()));
      corners_f_map.emplace_back(cv::Point3d(corner3_f_map.x(), corner3_f_map.y(), corner3_f_map.z()));
    }


    void append_corners_f_marker(double marker_length, std::vector<cv::Point3d> &corners_f_marker)
    {
      // Add to the list of the corner locations in the marker frame.
      corners_f_marker.emplace_back(cv::Point3d(-marker_length / 2.f, marker_length / 2.f, 0.f));
      corners_f_marker.emplace_back(cv::Point3d(marker_length / 2.f, marker_length / 2.f, 0.f));
      corners_f_marker.emplace_back(cv::Point3d(marker_length / 2.f, -marker_length / 2.f, 0.f));
      corners_f_marker.emplace_back(cv::Point3d(-marker_length / 2.f, -marker_length / 2.f, 0.f));
    }

    void append_corners_f_image(const Observation &observation, std::vector<cv::Point2f> &corners_f_image)
    {
      corners_f_image.emplace_back(
        cv::Point2f(static_cast<float>(observation.x0()), static_cast<float>(observation.y0())));
      corners_f_image.emplace_back(
        cv::Point2f(static_cast<float>(observation.x1()), static_cast<float>(observation.y1())));
      corners_f_image.emplace_back(
        cv::Point2f(static_cast<float>(observation.x2()), static_cast<float>(observation.y2())));
      corners_f_image.emplace_back(
        cv::Point2f(static_cast<float>(observation.x3()), static_cast<float>(observation.y3())));
    };

    Observations to_observations(const std::vector<int> &ids, const std::vector<std::vector<cv::Point2f>> &corners)
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

    tf2::Transform to_tf2_transform(const cv::Vec3d &rvec, const cv::Vec3d &tvec)
    {
      tf2::Vector3 t(tvec[0], tvec[1], tvec[2]);
      cv::Mat rmat;
      cv::Rodrigues(rvec, rmat);
      tf2::Matrix3x3 m;
      for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 3; col++) {
          m[row][col] = rmat.at<double>(row, col);  // Row- vs. column-major order
        }
      }
      tf2::Transform result(m, t);
      return result;
    }

    void to_cv_rvec_tvec(const TransformWithCovariance &t, cv::Vec3d &rvec, cv::Vec3d &tvec)
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
  };

// ==============================================================================
// FiducialMath::SamFiducialMath class
// ==============================================================================

  class FiducialMath::SamFiducialMath
  {
    CvFiducialMath &cv_;

    const gtsam::SharedNoiseModel corner_3D_constrained_noise_{
      gtsam::noiseModel::Constrained::MixedSigmas(gtsam::Z_3x1)};
    gtsam::Key camera_key_{gtsam::Symbol('c', 1)};


    class ResectioningFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3>
    {
      const boost::shared_ptr<gtsam::Cal3DS2> &cal3ds2_;
      const gtsam::Point3 P_;       ///< 3D point on the calibration rig
      const gtsam::Point2 p_;       ///< 2D measurement of the 3D point

    public:
      /// Construct factor given known point P and its projection p
      ResectioningFactor(const gtsam::SharedNoiseModel &model,
                         const gtsam::Key key,
                         const boost::shared_ptr<gtsam::Cal3DS2> &cal3ds2,
                         gtsam::Point2 p,
                         gtsam::Point3 P) :
        NoiseModelFactor1<gtsam::Pose3>(model, key),
        cal3ds2_{cal3ds2},
        P_(std::move(P)),
        p_(std::move(p))
      {}

      /// evaluate the error
      gtsam::Vector evaluateError(const gtsam::Pose3 &pose,
                                  boost::optional<gtsam::Matrix &> H) const override
      {
        auto camera = gtsam::PinholeCamera<gtsam::Cal3DS2>{pose, *cal3ds2_};
        return camera.project(P_, H) - p_;
      }
    };

  public:
    gtsam::Pose3 to_pose3(const tf2::Transform &transform)
    {
      auto q = transform.getRotation();
      auto t = transform.getOrigin();
      return gtsam::Pose3{gtsam::Rot3{q.w(), q.x(), q.y(), q.z()},
                          gtsam::Vector3{t.x(), t.y(), t.z()}};
    }

    gtsam::Matrix6 to_pose_cov_sam(const TransformWithCovariance::cov_type cov)
    {
      gtsam::Matrix6 cov_sam;
      for (int r = 0; r < 6; r += 1) {
        for (int c = 0; c < 6; c += 1) {
          static int ro[] = {3, 4, 5, 0, 1, 2};
          cov_sam(ro[r], ro[c]) = cov[r * 6 + c];
        }
      }
      return cov_sam;
    }

    gtsam::Point3 to_point3(const tf2::Vector3 &vector3)
    {
      return gtsam::Point3{vector3.x(), vector3.y(), vector3.z()};
    }

    gtsam::Matrix3 to_point_cov_sam(const Vector3WithCovariance::cov_type cov)
    {
      gtsam::Matrix3 cov_sam;
      for (int r = 0; r < 3; r += 1) {
        for (int c = 0; c < 3; c += 1) {
          cov_sam(r, c) = cov[r * 3 + c];
        }
      }
      return cov_sam;
    }

    TransformWithCovariance::cov_type to_pose_cov_type(const gtsam::Matrix6 &cov_sam)
    {
      // Convert covariance
      TransformWithCovariance::cov_type cov;
      for (int r = 0; r < 6; r += 1) {
        for (int c = 0; c < 6; c += 1) {
          static int ro[] = {3, 4, 5, 0, 1, 2};
          cov[r * 6 + c] = cov_sam(ro[r], ro[c]);
        }
      }
      return cov;
    }

    Vector3WithCovariance::cov_type to_point_cov_type(const gtsam::Matrix3 &cov_sam)
    {
      // Convert covariance
      Vector3WithCovariance::cov_type cov;
      for (int r = 0; r < 3; r += 1) {
        for (int c = 0; c < 3; c += 1) {
          cov[r * 3 + c] = cov_sam(r, c);
        }
      }
      return cov;
    }

    TransformWithCovariance to_transform_with_covariance(const gtsam::Pose3 &pose_sam, const gtsam::Matrix6 &cov_sam)
    {
      auto q1 = pose_sam.rotation().toQuaternion().coeffs();
      auto &t = pose_sam.translation();
      return TransformWithCovariance{
        tf2::Transform{tf2::Quaternion{q1[0], q1[1], q1[2], q1[3]},
                       tf2::Vector3{t.x(), t.y(), t.z()}},
        to_pose_cov_type(cov_sam)};
    }

    TransformWithCovariance extract_transform_with_covariance(const gtsam::NonlinearFactorGraph &graph,
                                                              const gtsam::Values &result,
                                                              gtsam::Key key)
    {
      gtsam::Marginals marginals(graph, result);
      return to_transform_with_covariance(result.at<gtsam::Pose3>(key),
                                          marginals.marginalCovariance(key));
    }

    Vector3WithCovariance to_vector3_with_covariance(const gtsam::Point3 &point_sam, const gtsam::Matrix3 &cov_sam)
    {
      return Vector3WithCovariance{
        tf2::Vector3{point_sam.x(), point_sam.y(), point_sam.z()},
        to_point_cov_type(cov_sam)};
    }

    Vector3WithCovariance extract_vector3_with_covariance(gtsam::NonlinearFactorGraph &graph,
                                                          const gtsam::Values &result,
                                                          gtsam::Key key)
    {
      gtsam::Marginals marginals(graph, result);
      return to_vector3_with_covariance(result.at<gtsam::Point3>(key),
                                        marginals.marginalCovariance(key));
    }

    TransformWithCovariance to_cov_f_world(const TransformWithCovariance &twc)
    {
      auto pose_sam = to_pose3(twc.transform());
      auto cov_sam = to_pose_cov_sam(twc.cov());

      // Rotate the covariance from the body frame to the world frame.
      gtsam::Matrix6 adjoint_map = pose_sam.AdjointMap();
      gtsam::Matrix6 cov_f_world = adjoint_map * cov_sam * adjoint_map.transpose();

      return to_transform_with_covariance(pose_sam, cov_f_world);
    }

    std::array<gtsam::Symbol, 4> to_corner_keys(int id)
    {
      return std::array<gtsam::Symbol, 4>{
        gtsam::Symbol{'i', static_cast<std::uint64_t>(id)},
        gtsam::Symbol{'j', static_cast<std::uint64_t>(id)},
        gtsam::Symbol{'k', static_cast<std::uint64_t>(id)},
        gtsam::Symbol{'l', static_cast<std::uint64_t>(id)}};
    }

    TransformWithCovariance solve_camera_f_marker(
      const Observation &observation,
      const CameraInfo &camera_info,
      double marker_length)
    {
      // 1. Allocate the graph and initial estimate
      gtsam::NonlinearFactorGraph graph{};
      gtsam::Values initial{};

      // 2. add factors to the graph
      std::vector<cv::Point3d> corners_f_marker{};
      std::vector<cv::Point2f> corners_f_image{};

      cv_.append_corners_f_marker(marker_length, corners_f_marker);
      cv_.append_corners_f_image(observation, corners_f_image);

      for (size_t j = 0; j < corners_f_image.size(); j += 1) {
        gtsam::Point2 corner_f_image{corners_f_image[j].x, corners_f_image[j].y};
        gtsam::Point3 corner_f_marker{corners_f_marker[j].x, corners_f_marker[j].y, corners_f_marker[j].z};
        graph.emplace_shared<ResectioningFactor>(
          gtsam::noiseModel::Isotropic::Sigma(2, cv_.cxt_.corner_measurement_sigma_),
          camera_key_,
          camera_info.cv()->cal3ds2(),
          corner_f_image,
          corner_f_marker);
      }

      // 3. Add the initial estimate for the camera pose in the marker frame
      auto cv_t_camera_marker = cv_.solve_t_camera_marker(observation, camera_info, marker_length);
      auto camera_f_marker_initial = to_pose3(cv_t_camera_marker.transform().inverse());
      initial.insert(camera_key_, camera_f_marker_initial);

      // 4. Optimize the graph using Levenberg-Marquardt
      auto result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();
//      std::cout << "initial error = " << graph.error(initial) << std::endl;
//      std::cout << "final error = " << graph.error(result) << std::endl;

      // 5. Extract the result
      return extract_transform_with_covariance(graph, result, camera_key_);
    }

  public:
    explicit SamFiducialMath(CvFiducialMath &cv) :
      cv_{cv}
    {}


    void load_graph_from_observations_sfm(const Observations &observations, const CameraInfo &camera_info, Map &map,
                                          const TransformWithCovariance &t_map_camera_initial,
                                          gtsam::Key camera_key, bool add_unknown_markers,
                                          gtsam::NonlinearFactorGraph &graph, gtsam::Values &initial)
    {
      // Add measurement factors, known marker priors, and marker initial estimates to the graph
      for (auto &observation : observations.observations()) {

        // See if this is a known marker by looking it up in the map.
        auto marker_ptr = map.find_marker(observation.id());

        // Add the measurement and initial value if this is a known marker or if we are supposed to add unknown markers.
        if (marker_ptr != nullptr || add_unknown_markers) {
          auto corner_keys = to_corner_keys(observation.id());

          // Get the measurements (image locations) for the 4 corner points.
          std::vector<cv::Point2f> corners_f_image{};
          cv_.append_corners_f_image(observation, corners_f_image);

          // Add these measurements as a general projection factor
          for (size_t j = 0; j < corners_f_image.size(); j += 1) {
            gtsam::Point2 corner_f_image{corners_f_image[j].x, corners_f_image[j].y};
            graph.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2>>(
              corner_f_image, gtsam::noiseModel::Isotropic::Sigma(2, cv_.cxt_.corner_measurement_sigma_),
              camera_key, corner_keys[j], camera_info.cv()->cal3ds2());
          }

          // Now add initial values and potentially priors.
          // But only if this marker has not been initialized before
          if (!initial.exists(corner_keys[0])) {

            if (marker_ptr != nullptr) {

              // Process a known marker.
              // If this is a fixed marker, then get the corner points from the
              // pose of the fixed marker.
              if (marker_ptr->is_fixed() || !marker_ptr->has_corners()) {
                // Get the corner locations from the map.
                std::vector<cv::Point3d> corners_f_map{};
                cv_.append_corners_f_map(marker_ptr->t_map_marker(), map.marker_length(), corners_f_map);

                // Add the initial value and the prior.
                for (size_t j = 0; j < corners_f_image.size(); j += 1) {
                  gtsam::Point3 corner_f_map{corners_f_map[j].x, corners_f_map[j].y, corners_f_map[j].z};
                  // Add the estimated value
                  initial.insert(corner_keys[j], corner_f_map);
                  // Add the prior with a constrained noise model
                  graph.emplace_shared<gtsam::PriorFactor<gtsam::Point3> >(
                    corner_keys[j], corner_f_map, corner_3D_constrained_noise_);
                }

              } else {
                // This is not a fixed marker, so the corners and their uncertainty have been stored in
                // the map. So get the corner priors and their covariance from the map and add them to the graph.
                for (size_t j = 0; j < corners_f_image.size(); j += 1) {
                  gtsam::Point3 corner_f_map{to_point3(marker_ptr->corners_f_map()[j].vector3())};

                  // Add the initial value
                  initial.insert(corner_keys[j], corner_f_map);

                  // Add the prior and the noise model
                  auto corner_cov = to_point_cov_sam(marker_ptr->corners_f_map()[j].cov());
                  graph.emplace_shared<gtsam::PriorFactor<gtsam::Point3> >(
                    corner_keys[j], corner_f_map, gtsam::noiseModel::Gaussian::Covariance(corner_cov));
                }
              }

            } else {
              // Process an unknown marker. Somehow we have to figure out it's initial
              // estimate. Do this by using OpenCV to figure t_camera_marker, then using
              // t_map_camera_initial to find t_map_marker and then using append_corners_f_map()
              // to get the corner locations.

              auto cv_t_camera_marker = cv_.solve_t_camera_marker(observation, camera_info, map.marker_length());
              auto cv_t_map_marker = TransformWithCovariance{
                t_map_camera_initial.transform() * cv_t_camera_marker.transform()};

              // Get the corner locations in the map frame.
              std::vector<cv::Point3d> corners_f_map{};
              cv_.append_corners_f_map(cv_t_map_marker, map.marker_length(), corners_f_map);

              for (size_t j = 0; j < corners_f_image.size(); j += 1) {
                gtsam::Point3 corner_f_map{corners_f_map[j].x, corners_f_map[j].y, corners_f_map[j].z};
                // Add the estimated value
                initial.insert(corner_keys[j], corner_f_map);
              }
            }
          }
        }
      }

      // Add the camera initial value.
      initial.insert(camera_key, to_pose3(t_map_camera_initial.transform()));
    }

    TransformWithCovariance solve_t_map_camera_sfm(const Observations &observations,
                                                   const CameraInfo &camera_info,
                                                   Map &map)
    {
      // Get an estimate of camera_f_map.
      auto cv_t_map_camera = cv_.solve_t_map_camera(observations, camera_info, map);

      // If we could not find an estimate, then there are no known markers in the image.
      if (!cv_t_map_camera.is_valid()) {
        return cv_t_map_camera;
      }

      // 1. Allocate the graph and initial estimate
      gtsam::NonlinearFactorGraph graph{};
      gtsam::Values initial{};

      // 2. add factors to the graph
      load_graph_from_observations_sfm(observations, camera_info, map,
                                       cv_t_map_camera, camera_key_, false,
                                       graph, initial);

      // 4. Optimize the graph using Levenberg-Marquardt
      auto result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();
//      std::cout << "initial error = " << graph.error(initial) << std::endl;
//      std::cout << "final error = " << graph.error(result) << std::endl;

      // 5. Extract the result
      auto t_map_camera = extract_transform_with_covariance(graph, result, camera_key_);

      // 6. Rotate the covariance into the world frame
      return to_cov_f_world(t_map_camera);
    }

    void load_graph_from_observations_slam(const Observations &observations, const CameraInfo &camera_info, Map &map,
                                           const TransformWithCovariance &t_map_camera_initial,
                                           gtsam::Key camera_key, bool add_unknown_markers, bool add_non_fixed_priors,
                                           gtsam::NonlinearFactorGraph &graph, gtsam::Values &initial)
    {

      // Add measurement factors, known marker priors, and marker initial estimates to the graph.
      for (auto &observation : observations.observations()) {

        // See if this is a known marker by looking it up in the map.
        auto marker_ptr = map.find_marker(observation.id());

        // nothing to do if the marker is unknown and we are not adding unknown markers.
        if (marker_ptr == nullptr && !add_unknown_markers) {
          continue;
        }

        // Add the between measurement, the initial value, and add it as a prior.
        gtsam::Symbol marker_key{'m', static_cast<std::uint64_t>(observation.id())};

        // Get the measurement
        auto camera_f_marker = solve_camera_f_marker(observation, camera_info, map.marker_length());

        // Add the between factor for this measurement
        auto cov = to_pose_cov_sam(camera_f_marker.cov());
        graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
          marker_key,
          camera_key,
          to_pose3(camera_f_marker.transform()),
          gtsam::noiseModel::Gaussian::Covariance(cov));

        // If the initial value already exists, then don't add it or potentially a prior.
        if (initial.exists(marker_key)) {
          continue;
        }

        // A known marker.
        if (marker_ptr != nullptr) {
          // Get the pose from a marker in the map and add it as the initial value.
          auto known_marker_f_map = to_pose3(marker_ptr->t_map_marker().transform());
          initial.insert(marker_key, known_marker_f_map);

          // Only add priors if this is a fixed marker or if requested.
          if (marker_ptr->is_fixed() || add_non_fixed_priors) {
            auto known_marker_cov = to_pose_cov_sam(marker_ptr->t_map_marker().cov());

            // Choose the noise model to use for the marker pose prior. Choose between
            // the covariance stored with the marker in the map or just a constrained model
            // that indicates that the marker pose is known precisely.
            // Use the constrained model if:
            //  the marker is fixed -> The location of the marker is known precisely
            //  or the map_style > MapStyles::pose -> there are no valid covariances
            //  or the first variance is zero -> A shortcut that says there is no variance.
            bool use_constrained = marker_ptr->is_fixed() ||
                                   map.map_style() == Map::MapStyles::pose ||
                                   known_marker_cov(0, 0) == 0.0;

            // Create the appropriate marker pose prior noise model.
            auto known_noise_model = use_constrained ?
                                     gtsam::noiseModel::Constrained::MixedSigmas(gtsam::Z_6x1) :
                                     gtsam::noiseModel::Gaussian::Covariance(known_marker_cov);

            // Add the prior for the known marker.
            graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3> >(marker_key,
                                                                    known_marker_f_map,
                                                                    known_noise_model);
          }

        } else {
          // An unknown marker.
          auto unknown_marker_f_map = t_map_camera_initial.transform() * camera_f_marker.transform().inverse();
          initial.insert(marker_key, to_pose3(unknown_marker_f_map));
        }
      }

      // Add the camera initial value.
      initial.insert(camera_key, to_pose3(t_map_camera_initial.transform()));
    }

    TransformWithCovariance solve_t_map_camera_slam(const Observations &observations,
                                                    const CameraInfo &camera_info,
                                                    Map &map)
    {
      // Get an estimate of camera_f_map.
      auto cv_t_map_camera = cv_.solve_t_map_camera(observations, camera_info, map);

      // If we could not find an estimate, then there are no known markers in the image.
      if (!cv_t_map_camera.is_valid()) {
        return cv_t_map_camera;
      }

      // 1. Allocate the graph and initial estimate
      gtsam::NonlinearFactorGraph graph{};
      gtsam::Values initial{};

      // 2. add factors to the graph
      load_graph_from_observations_slam(
        observations,
        camera_info,
        map, cv_t_map_camera,
        camera_key_, false, true,
        graph, initial);

      // 4. Optimize the graph using Levenberg-Marquardt
      auto result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();
//      std::cout << "initial error = " << graph.error(initial) << std::endl;
//      std::cout << "final error = " << graph.error(result) << std::endl;

      // 5. Extract the result
      auto t_map_camera = extract_transform_with_covariance(graph, result, camera_key_);

      // 6. Rotate the covariance into the world frame
      return to_cov_f_world(t_map_camera);
    }
  };

// ==============================================================================
// FiducialMath::UpdateFiducialMath class
// ==============================================================================

  class FiducialMath::UpdateFiducialMath
  {
    CvFiducialMath &cv_;
    SamFiducialMath &sam_;

    // These parameters are snap-shotted when the class is constructed. This allows
    // the map creation to proceed in one mode.
    FiducialMathContext cxt_;

    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values initial_;
    gtsam::ISAM2 isam2_;
    std::uint64_t frames_processeed_;
    std::map<int, int> marker_seen_counts_{};

    static gtsam::ISAM2Params get_isam2_params(FiducialMathContext &cxt)
    {
      gtsam::ISAM2Params isam2_params;
      isam2_params.relinearizeThreshold = 0.01;
      isam2_params.relinearizeSkip = 1;
      return isam2_params;
    }

    void update_marker_seen_counts(const Observations &observations)
    {
      // Record the markers that were added to the graph
      for (auto &observation : observations.observations()) {
        auto pair = marker_seen_counts_.find(observation.id());
        if (pair == marker_seen_counts_.end()) {
          marker_seen_counts_.insert(std::pair<int, int>{observation.id(), 1});
        } else {
          pair->second += 1;
        }
      }
    }

    class TransformFromFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3>
    {
      typedef NoiseModelFactor1 <gtsam::Pose3> Base;

      const gtsam::Point3 corner_f_marker_;
      const gtsam::Point3 corner_f_world_;

    public:

      /// Construct factor the corner coordinates in the marker and world frames
      TransformFromFactor(const gtsam::SharedNoiseModel &model, const gtsam::Key &key,
                          gtsam::Point3 corner_f_marker, gtsam::Point3 corner_f_world) :
        Base(model, key), corner_f_marker_{std::move(corner_f_marker)}, corner_f_world_{std::move(corner_f_world)}
      {}

      /// evaluate the error
      gtsam::Vector evaluateError(const gtsam::Pose3 &pose,
                                  boost::optional<gtsam::Matrix &> H = boost::none) const override
      {
        return pose.transformFrom(corner_f_marker_, H) - corner_f_world_;
      }
    };

    TransformWithCovariance t_map_marker_from_corners(
      const std::vector<Vector3WithCovariance> &v2wc_corners_f_map,
      double marker_length)
    {
      gtsam::Symbol pose_key{'p', 0};
      auto corners_f_marker = Convert::corners_f_marker<gtsam::Point3>(marker_length);
      auto corners_f_map = Convert::to_corner_points<gtsam::Point3>(v2wc_corners_f_map);

      /* 1. create graph */
      gtsam::NonlinearFactorGraph graph;
      gtsam::Values initial;

      /* 2. add measurement factors to the graph */
      for (int i = 0; i < corners_f_map.size(); i += 1) {
        auto cov = sam_.to_point_cov_sam(v2wc_corners_f_map[i].cov());
        graph.emplace_shared<TransformFromFactor>(
          gtsam::noiseModel::Gaussian::Covariance(cov),
          pose_key,
          corners_f_marker[i],
          corners_f_map[i]);
      }

      /* 3. Create an initial estimate for the camera pose */
      auto &cfm = corners_f_map;
      auto t = (cfm[0] + cfm[1] + cfm[2] + cfm[3]) / 4.;
      auto x_axis = ((cfm[1] + cfm[2]) / 2. - t).normalized();
      auto z_axis = x_axis.cross(cfm[1] - t).normalized();
      auto y_axis = z_axis.cross(x_axis);
      auto r = gtsam::Rot3{(gtsam::Matrix3{} << x_axis, y_axis, z_axis).finished()};
      auto camera_f_world_initial = gtsam::Pose3{r, t};
      initial.insert(pose_key, camera_f_world_initial);

      /* 4. Optimize the graph using Levenberg-Marquardt*/
      auto result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();

      auto marker_f_map = result.at<gtsam::Pose3>(pose_key);

      gtsam::Marginals marginals(graph, result);
      gtsam::Matrix6 marker_f_map_covariance = marginals.marginalCovariance(pose_key);

      return sam_.to_transform_with_covariance(marker_f_map, marker_f_map_covariance);
    }

    void update_map_sfm(const Observations &observations,
                        const CameraInfo &camera_info,
                        Map &map)
    {
      // Get an estimate of camera_f_map.
      auto cv_t_map_camera_initial = cv_.solve_t_map_camera(observations, camera_info, map);

      // Have to have a valid camera pose and see at least two markers before this routine can do anything.
      if (!cv_t_map_camera_initial.is_valid() || observations.size() < 2) {
        return;
      }

      // Crate a camera key based on the frame count and then update
      // the frame count.
      gtsam::Symbol camera_key{'c', frames_processeed_};
      frames_processeed_ += 1;

      sam_.load_graph_from_observations_sfm(observations, camera_info, map,
                                            cv_t_map_camera_initial,
                                            camera_key, true,
                                            graph_, initial_);

      // Record the markers that were added to the graph
      update_marker_seen_counts(observations);
    }

    void update_map_for_publishing_sfm(Map &map)
    {
      if (marker_seen_counts_.size() < 2) {
        return;
      }

      // Now optimize this graph
      auto result = gtsam::LevenbergMarquardtOptimizer(graph_, initial_).optimize();
      std::cout << "initial error = " << graph_.error(initial_) << std::endl;
      std::cout << "final error = " << graph_.error(result) << std::endl;

//      graph_.print("Graph\n");
//      initial_.print("Initial\n");
      result.print("Result\n");

      // Update the map
      for (auto &pair : marker_seen_counts_) {
        auto corner_keys = sam_.to_corner_keys(pair.first);

        std::vector<Vector3WithCovariance> corners_f_map{};
        for (const auto &corner_key : corner_keys) {
          corners_f_map.emplace_back(
            sam_.extract_vector3_with_covariance(graph_, result, corner_key));
        }

        auto t_map_marker = t_map_marker_from_corners(corners_f_map, map.marker_length());

//        // update an existing marker or add a new one.
//        auto marker_ptr = map.find_marker(pair.first);
//        if (marker_ptr == nullptr) {
//          map.add_marker(Marker{pair.first, t_map_marker, corners_f_map});
//        } else if (!marker_ptr->is_fixed()) {
//          marker_ptr->set_t_map_marker(t_map_marker);
//          marker_ptr->set_corners_f_map(corners_f_map);
//          marker_ptr->set_update_count(marker_ptr->update_count() + 1);
//        }

        std::cout << "M " << pair.first << " "
                  << t_map_marker.mu()[0] << " "
                  << t_map_marker.mu()[1] << " "
                  << t_map_marker.mu()[2] << " "
                  << t_map_marker.mu()[3] << " "
                  << t_map_marker.mu()[4] << " "
                  << t_map_marker.mu()[5] << " "
                  << std::endl;
      }
    }

    void update_markers_in_map(int marker_id,
                               const gtsam::NonlinearFactorGraph &graph,
                               const gtsam::Values &result,
                               Map &map)
    {
      gtsam::Symbol marker_key{'m', static_cast<std::uint64_t>(marker_id)};
      auto t_map_marker = sam_.extract_transform_with_covariance(graph, result, marker_key);

      // update an existing marker or add a new one.
      auto marker_ptr = map.find_marker(marker_id);
      if (marker_ptr == nullptr) {
        map.add_marker(Marker{marker_id, t_map_marker});
      } else if (!marker_ptr->is_fixed()) {
        marker_ptr->set_t_map_marker(t_map_marker);
        marker_ptr->set_update_count(marker_ptr->update_count() + 1);
      }
    }

    void update_map_slam(const Observations &observations,
                         const CameraInfo &camera_info,
                         Map &map)
    {
      // Get an estimate of camera_f_map.
      auto cv_t_map_camera_initial = cv_.solve_t_map_camera(observations, camera_info, map);

      // Have to have a valid camera pose and see at least two markers before this routine can do anything.
      if (!cv_t_map_camera_initial.is_valid() || observations.size() < 2) {
        return;
      }

      // Crate a camera key based on the frame count and then update
      // the frame count.
      gtsam::Symbol camera_key{'c', frames_processeed_};
      frames_processeed_ += 1;

      gtsam::NonlinearFactorGraph graph{};
      gtsam::Values initial{};

      sam_.load_graph_from_observations_slam(observations, camera_info, map,
                                             cv_t_map_camera_initial,
                                             camera_key, true, true,
                                             graph, initial);

      // Now optimize this graph
      auto result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();
//      std::cout << "initial error = " << graph.error(initial) << std::endl;
//      std::cout << "final error = " << graph.error(result) << std::endl;

      // Update the map based on the results of this single frame optimization.
      for (auto &observation : observations.observations()) {
        update_markers_in_map(observation.id(), graph, result, map);
      }

      // If we are doing multi-frame optimization, add data to the global graph.
      if (cv_.cxt_.multi_frame_optimization_) {

        sam_.load_graph_from_observations_slam(observations, camera_info, map,
                                               cv_t_map_camera_initial,
                                               camera_key, true, false,
                                               graph_, initial_);

        // Record the markers that have been added to the multi-frame optimization graph
        update_marker_seen_counts(observations);
      }
    }

    void update_map_for_publishing_slam(Map &map)
    {
      if (!cv_.cxt_.multi_frame_optimization_ || marker_seen_counts_.size() < 2) {
        return;
      }

      // Now optimize this multi-frame graph
      auto params = gtsam::LevenbergMarquardtParams();
      params.setVerbosityLM("TERMINATION");
      params.setVerbosity("TERMINATION");
      params.setRelativeErrorTol(1e-8);
      params.setAbsoluteErrorTol(1e-8);
      auto result = gtsam::LevenbergMarquardtOptimizer(graph_, initial_, params).optimize();
      std::cout << "initial error = " << graph_.error(initial_) << std::endl;
      std::cout << "final error = " << graph_.error(result) << std::endl;
      std::cout << "frames = " << frames_processeed_ << std::endl;

      // Push the multi frame optimized marker poses into the map.
      for (auto &pair : marker_seen_counts_) {
        update_markers_in_map(pair.first, graph_, result, map);
      }
    }

    void update_map_cv(const Observations &observations,
                       const CameraInfo &camera_info,
                       Map &map)
    {
      // Get an estimate of camera_f_map.
      auto t_map_camera = cv_.solve_t_map_camera(observations, camera_info, map);

      // For all observations estimate the marker location and update the map
      for (auto &observation : observations.observations()) {

        auto t_camera_marker = cv_.solve_t_camera_marker(observation, camera_info, map.marker_length());
        auto t_map_marker = TransformWithCovariance(t_map_camera.transform() * t_camera_marker.transform());

        // Update an existing marker or add a new one.
        auto marker_ptr = map.find_marker(observation.id());
        if (marker_ptr) {
          auto &marker = *marker_ptr;
          cv_.update_marker_simple_average(marker, t_map_marker);

        } else {
          map.add_marker(Marker(observation.id(), t_map_marker));
        }
      }
    }

  public:
    UpdateFiducialMath(CvFiducialMath &cv,
                       SamFiducialMath &sam) :
      cv_{cv}, sam_{sam}, cxt_{cv_.cxt_},
      graph_{}, initial_{},
      isam2_{get_isam2_params(cv.cxt_)},
      frames_processeed_{0}
    {

    }

    void update_map(const Observations &observations,
                    const CameraInfo &camera_info,
                    Map &map)
    {
      if (cxt_.sam_not_cv_) {
        if (cxt_.sfm_not_slam_) {
          update_map_sfm(observations, camera_info, map);
        } else {
          update_map_slam(observations, camera_info, map);
        }
      } else {
        update_map_cv(observations, camera_info, map);
      }
    }

    void update_map_for_publishing(Map &map)
    {
      if (cxt_.sam_not_cv_) {
        if (cxt_.sfm_not_slam_) {
          update_map_for_publishing_sfm(map);
        } else {
          update_map_for_publishing_slam(map);
        }
      }
    }

    std::string update_map_cmd(std::string &cmd)
    {
      if (cmd == "start") {
        return ros2_shared::string_printf(
          "Start map creation\nsam_not_cv %d\nsfm_not_slam %d\nmulti_frame %d",
          cxt_.sam_not_cv_, cxt_.sfm_not_slam_, cxt_.multi_frame_optimization_);
      }
      return std::string{};
    }
  };

// ==============================================================================
// FiducialMath class
// ==============================================================================

  FiducialMath::FiducialMath(FiducialMathContext &cxt) :
    cv_{std::make_unique<CvFiducialMath>(cxt)},
    sam_{std::make_unique<SamFiducialMath>(*cv_)},
    update_{}
  {}

  FiducialMath::~FiducialMath() = default;

  TransformWithCovariance FiducialMath::solve_t_camera_marker(
    const Observation &observation,
    const CameraInfo &camera_info,
    double marker_length)
  {
    return cv_->solve_t_camera_marker(observation, camera_info, marker_length);
  }

  TransformWithCovariance FiducialMath::solve_t_map_camera(const Observations &observations,
                                                           const CameraInfo &camera_info,
                                                           Map &map)
  {
    return !cv_->cxt_.sam_not_cv_ ?
           cv_->solve_t_map_camera(observations, camera_info, map) :
           cv_->cxt_.sfm_not_slam_ ?
           sam_->solve_t_map_camera_sfm(observations, camera_info, map) :
           sam_->solve_t_map_camera_slam(observations, camera_info, map);
  }

  Observations FiducialMath::detect_markers(std::shared_ptr<cv_bridge::CvImage> &color,
                                            std::shared_ptr<cv_bridge::CvImage> &color_marked)
  {
    return cv_->detect_markers(color, color_marked);
  }

  void FiducialMath::annotate_image_with_marker_axis(std::shared_ptr<cv_bridge::CvImage> &color_marked,
                                                     const TransformWithCovariance &t_camera_marker,
                                                     const CameraInfo &camera_info)
  {
    cv_->annotate_image_with_marker_axis(color_marked, t_camera_marker, camera_info);
  }

  void FiducialMath::update_map(const Observations &observations,
                                const CameraInfo &camera_info,
                                Map &map)
  {
    if (update_) {
      update_->update_map(observations, camera_info, map);
    }
  }

  void FiducialMath::update_map_for_publishing(Map &map)
  {
    if (update_) {
      update_->update_map_for_publishing(map);
    }
  }

  std::string FiducialMath::update_map_cmd(std::string &cmd)
  {
    if (update_) {
      auto ret_str = update_->update_map_cmd(cmd);
      if (cmd == "done") {
        update_.reset(nullptr);
      }
      return ret_str;
    }

    if (cmd == "start") {
      update_ = std::make_unique<UpdateFiducialMath>(*cv_, *sam_);
      return update_->update_map_cmd(cmd);
    }

    return std::string("No Update active");
  }
}
