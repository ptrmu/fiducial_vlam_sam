
#include "fiducial_math.hpp"

#include "map.hpp"
#include "observation.hpp"
#include "transform_with_covariance.hpp"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/aruco.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include "gtsam/inference/Symbol.h"
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

namespace fiducial_vlam
{
// ==============================================================================
// CameraInfo::CvCameraInfo class
// ==============================================================================

  class CameraInfo::CvCameraInfo
  {
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

  public:
    CvCameraInfo() = delete;

    explicit CvCameraInfo(const sensor_msgs::msg::CameraInfo &msg)
      : camera_matrix_(3, 3, CV_64F, 0.), dist_coeffs_(1, 5, CV_64F)
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
    const CameraInfo ci_;

    explicit CvFiducialMath(const CameraInfo &camera_info)
      : ci_{camera_info}
    {}

    explicit CvFiducialMath(const sensor_msgs::msg::CameraInfo &camera_info_msg)
      : ci_(camera_info_msg)
    {}

    TransformWithCovariance solve_t_camera_marker(
      const Observation &observation,
      double marker_length)
    {
      // Build up two lists of corner points: 2D in the image frame, 3D in the marker frame
      std::vector<cv::Point3d> corners_f_marker;
      std::vector<cv::Point2f> corners_f_image;

      append_corners_f_marker(corners_f_marker, marker_length);
      append_corners_f_image(corners_f_image, observation);

      // Figure out image location.
      cv::Vec3d rvec, tvec;
      cv::solvePnP(corners_f_marker, corners_f_image,
                   ci_.cv()->camera_matrix(), ci_.cv()->dist_coeffs(),
                   rvec, tvec);

      // rvec, tvec output from solvePnp "brings points from the model coordinate system to the
      // camera coordinate system". In this case the marker frame is the model coordinate system.
      // So rvec, tvec are the transformation t_camera_marker.
      return TransformWithCovariance(to_tf2_transform(rvec, tvec));
    }

    TransformWithCovariance solve_t_map_camera(const Observations &observations,
                                               const std::vector<TransformWithCovariance> &t_map_markers,
                                               double marker_length)
    {
      // Build up two lists of corner points: 2D in the image frame, 3D in the marker frame
      std::vector<cv::Point3d> all_corners_f_map;
      std::vector<cv::Point2f> all_corners_f_image;

      for (int i = 0; i < observations.size(); i += 1) {
        auto &observation = observations.observations()[i];
        auto &t_map_marker = t_map_markers[i];
        if (t_map_marker.is_valid()) {
          append_corners_f_map(all_corners_f_map, t_map_marker, marker_length);
          append_corners_f_image(all_corners_f_image, observation);
        }
      }

      // If there are no known markers in the observation set, then don't
      // try to find the camera position
      if (all_corners_f_map.empty()) {
        return TransformWithCovariance{};
      }

      // Figure out camera location.
      cv::Vec3d rvec, tvec;
      cv::solvePnP(all_corners_f_map, all_corners_f_image,
                   ci_.cv()->camera_matrix(), ci_.cv()->dist_coeffs(),
                   rvec, tvec);

      // For certain cases, there is a chance that the multi marker solvePnP will
      // return the mirror of the correct solution. So try solvePn[Ransac as well.
      if (all_corners_f_image.size() > 1 * 4 && all_corners_f_image.size() < 4 * 4) {
        cv::Vec3d rvecRansac, tvecRansac;
        cv::solvePnPRansac(all_corners_f_map, all_corners_f_image,
                           ci_.cv()->camera_matrix(), ci_.cv()->dist_coeffs(),
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

      if (tvec[0] < 0) { // specific tests for bad pose determination
        int xxx = 9;
      }

      // rvec, tvec output from solvePnp "brings points from the model coordinate system to the
      // camera coordinate system". In this case the map frame is the model coordinate system.
      // So rvec, tvec are the transformation t_camera_map.
      auto tf2_t_map_camera = to_tf2_transform(rvec, tvec).inverse();
      return TransformWithCovariance(tf2_t_map_camera);
    }

    Observations detect_markers(cv_bridge::CvImagePtr &color,
                                std::shared_ptr<cv_bridge::CvImage> &color_marked)
    {
      // Todo: make the dictionary a parameter
      auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
      auto detectorParameters = cv::aruco::DetectorParameters::create();
#if (CV_VERSION_MAJOR == 4)
      // Use the new AprilTag 2 corner algorithm, much better but much slower
      detectorParameters->cornerRefinementMethod = cv::aruco::CornerRefineMethod::CORNER_REFINE_APRILTAG;
#else
      detectorParameters->doCornerRefinement = true;
#endif

      // Color to gray for detection
      cv::Mat gray;
      cv::cvtColor(color->image, gray, cv::COLOR_BGR2GRAY);

      // Detect markers
      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f>> corners;
      cv::aruco::detectMarkers(gray, dictionary, corners, ids, detectorParameters);

      // Annotate the markers
      if (color_marked) {
        drawDetectedMarkers(color_marked->image, corners, ids);
      }

      // return the corners as a list of observations
      return to_observations(ids, corners);
    }

    void annotate_image_with_marker_axis(std::shared_ptr<cv_bridge::CvImage> &color_marked,
                                         const TransformWithCovariance &t_camera_marker)
    {
      cv::Vec3d rvec;
      cv::Vec3d tvec;
      to_cv_rvec_tvec(t_camera_marker, rvec, tvec);

      cv::aruco::drawAxis(color_marked->image,
                          ci_.cv()->camera_matrix(), ci_.cv()->dist_coeffs(),
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

    void update_map(const TransformWithCovariance &t_map_camera,
                    const Observations &observations,
                    Map &map)
    {
      // For all observations estimate the marker location and update the map
      for (auto &observation : observations.observations()) {

        auto t_camera_marker = solve_t_camera_marker(observation, map.marker_length());
        auto t_map_marker = TransformWithCovariance(t_map_camera.transform() * t_camera_marker.transform());

        // Update an existing marker or add a new one.
        auto marker_ptr = map.find_marker(observation.id());
        if (marker_ptr) {
          auto &marker = *marker_ptr;
          update_marker_simple_average(marker, t_map_marker);

        } else {
          map.add_marker(Marker(observation.id(), t_map_marker));
        }
      }

    }

    std::vector<cv::Point3d> get_corners_f_map(const TransformWithCovariance &t_map_marker,
                                               double marker_length)
    {
      std::vector<cv::Point3d> corners_f_map{};
      append_corners_f_map(corners_f_map,
                           t_map_marker,
                           marker_length);
      return corners_f_map;
    }

    std::vector<cv::Point2f> get_corners_f_image(const Observation &observation)
    {
      std::vector<cv::Point2f> corners_f_image{};
      append_corners_f_image(corners_f_image, observation);
      return corners_f_image;
    }

  private:
    void append_corners_f_map(std::vector<cv::Point3d> &corners_f_map,
                              const TransformWithCovariance &t_map_marker,
                              double marker_length)
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


    void append_corners_f_marker(std::vector<cv::Point3d> &corners_f_marker, double marker_length)
    {
      // Add to the list of the corner locations in the marker frame.
      corners_f_marker.emplace_back(cv::Point3d(-marker_length / 2.f, marker_length / 2.f, 0.f));
      corners_f_marker.emplace_back(cv::Point3d(marker_length / 2.f, marker_length / 2.f, 0.f));
      corners_f_marker.emplace_back(cv::Point3d(marker_length / 2.f, -marker_length / 2.f, 0.f));
      corners_f_marker.emplace_back(cv::Point3d(-marker_length / 2.f, -marker_length / 2.f, 0.f));
    }

    void append_corners_f_image(std::vector<cv::Point2f> &corners_f_image, const Observation &observation)
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

  public:
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
    gtsam::Cal3DS2 cal3ds2_;

    gtsam::NonlinearFactorGraph graph_{};
    gtsam::Values initial_{};
    gtsam::Key X1_{gtsam::Symbol('x', 1)};


    class ResectioningFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3>
    {
      const gtsam::Cal3DS2 cal3ds2_;
      const gtsam::Point3 P_;       ///< 3D point on the calibration rig
      const gtsam::Point2 p_;       ///< 2D measurement of the 3D point

    public:
      /// Construct factor given known point P and its projection p
      ResectioningFactor(const gtsam::SharedNoiseModel &model,
                         const gtsam::Key key,
                         const gtsam::Cal3DS2 &cal3ds2,
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
        auto camera = gtsam::PinholeCamera<gtsam::Cal3DS2>{pose, cal3ds2_};
        return camera.project(P_, H) - p_;
      }
    };

    TransformWithCovariance calc_cv_t_map_camera(const Observations &observations,
                                                 const std::vector<TransformWithCovariance> &t_map_markers,
                                                 double marker_length)
    {
      // Run through the observations and find one that refers to a marker
      // with known pose. Then use opencv's SolvePnp to find the pose
      // of the camera.
      int i_found = -1;
      for (int i = 0; i < observations.size(); i += 1) {
        auto &t_map_marker = t_map_markers[i];
        if (t_map_marker.is_valid()) {
          i_found = i;
          break;
        }
      }

      // If there were no known markers, then we can't localize the camera.
      if (i_found == -1) {
        return TransformWithCovariance{};
      }

      // Figure out camera location.
      cv::Vec3d rvec, tvec;
      cv::solvePnP(cv_.get_corners_f_map(t_map_markers[i_found], marker_length),
                   cv_.get_corners_f_image(observations.observations()[i_found]),
                   cv_.ci_.cv()->camera_matrix(),
                   cv_.ci_.cv()->dist_coeffs(),
                   rvec, tvec);

      // rvec, tvec output from solvePnp "brings points from the model coordinate system to the
      // camera coordinate system". In this case the map frame is the model coordinate system.
      // So rvec, tvec are the transformation t_camera_map.
      auto tf2_t_map_camera = cv_.to_tf2_transform(rvec, tvec).inverse();
      return TransformWithCovariance(tf2_t_map_camera);
    }

  public:
    explicit SamFiducialMath(CvFiducialMath &cv) :
      cv_{cv}, cal3ds2_{
      cv.ci_.cv()->camera_matrix().at<double>(0, 0),  // fx
      cv.ci_.cv()->camera_matrix().at<double>(1, 1),  // fy
      1.0, // s
      cv.ci_.cv()->camera_matrix().at<double>(0, 2),  // u0
      cv.ci_.cv()->camera_matrix().at<double>(1, 2),  // v0
      cv.ci_.cv()->dist_coeffs().at<double>(0), // k1
      cv.ci_.cv()->dist_coeffs().at<double>(1), // k2
      cv.ci_.cv()->dist_coeffs().at<double>(2), // p1
      cv.ci_.cv()->dist_coeffs().at<double>(3)} // p2
    {}

    TransformWithCovariance solve_t_camera_marker(
      const Observation &observation,
      double marker_length)
    {
      return TransformWithCovariance{};
    }

    TransformWithCovariance solve_t_map_camera(const Observations &observations,
                                               const std::vector<TransformWithCovariance> &t_map_markers,
                                               double marker_length)
    {
      // Get an estimate of camera_f_map.
      auto cv_t_map_camera = calc_cv_t_map_camera(observations,
                                                  t_map_markers,
                                                  marker_length);

      // If we could not find an estimate, then there are no known markers in the image.
      if (!cv_t_map_camera.is_valid()) {
        return cv_t_map_camera;
      }

      // Do the GTSAM camera resectioning process.
      // 1. clear the graph and initial estimate
      graph_.resize(0);
      initial_.clear();

      // 2. add factors to the graph
      const gtsam::SharedNoiseModel measurement_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(1, 1));
      for (int i = 0; i < observations.size(); i += 1) {
        auto &t_map_marker = t_map_markers[i];
        if (t_map_marker.is_valid()) {

          auto corners_f_map{cv_.get_corners_f_map(t_map_markers[i], marker_length)};
          auto corners_f_image{cv_.get_corners_f_image(observations.observations()[i])};

          for (size_t j = 0; j < corners_f_image.size(); j += 1) {
            gtsam::Point2 corner_f_image{corners_f_image[j].x, corners_f_image[j].y};
            gtsam::Point3 corner_f_map{corners_f_map[j].x, corners_f_map[j].y, corners_f_map[j].z};
            graph_.emplace_shared<ResectioningFactor>(measurement_noise, X1_,
                                                      cal3ds2_,
                                                      corner_f_image,
                                                      corner_f_map);
          }
        }
      }

      // 3. Add the initial estimate for the camera pose
      auto q = cv_t_map_camera.transform().getRotation();
      auto t = cv_t_map_camera.transform().getOrigin();
      gtsam::Pose3 initial_t_map_camera{gtsam::Rot3{q.w(), q.x(), q.y(), q.z()},
                                        gtsam::Vector3{t.x(), t.y(), t.z()}};
      initial_.insert(X1_, initial_t_map_camera);

      /* 4. Optimize the graph using Levenberg-Marquardt*/
      auto result = gtsam::LevenbergMarquardtOptimizer(graph_, initial_).optimize();
//      std::cout << "initial error = " << graph_.error(initial_) << std::endl;
//      std::cout << "final error = " << graph_.error(result) << std::endl;

      // 5. Get the answer
      auto camera_f_marker = result.at<gtsam::Pose3>(X1_);
      gtsam::Marginals marginals(graph_, result);
      auto camera_f_marker_covariance = marginals.marginalCovariance(X1_);

      // Convert covariance
      TransformWithCovariance::cov_type cov;
      for (int r = 0; r < 6; r += 1) {
        for (int c = 0; c < 6; c += 1) {
          static int ro[] = {3, 4, 5, 0, 1, 2};
          cov[r * 6 + c] = camera_f_marker_covariance(ro[r], ro[c]);
        }
      }

      auto q1 = camera_f_marker.rotation().toQuaternion().coeffs();
      auto &t1 = camera_f_marker.translation();
      TransformWithCovariance sam_t_map_camera{
        tf2::Transform{tf2::Quaternion{q1[0], q1[1], q1[2], q1[3]},
                       tf2::Vector3{t1.x(), t1.y(), t1.z()}},
        cov};

      // For now just return the approximate camera pose.
      return sam_t_map_camera;
    }
  };

// ==============================================================================
// FiducialMath class
// ==============================================================================

  FiducialMath::FiducialMath(const CameraInfo &camera_info) :
    cv_{std::make_unique<CvFiducialMath>(camera_info)},
    sam_{std::make_unique<SamFiducialMath>(*cv_)}
  {}

  FiducialMath::FiducialMath(const sensor_msgs::msg::CameraInfo &camera_info_msg) :
    cv_{std::make_unique<CvFiducialMath>(camera_info_msg)},
    sam_{std::make_unique<SamFiducialMath>(*cv_)}
  {}

  FiducialMath::~FiducialMath() = default;

  TransformWithCovariance FiducialMath::solve_t_camera_marker(
    const Observation &observation,
    double marker_length)
  {
    return cv_->solve_t_camera_marker(observation, marker_length);
  }

  TransformWithCovariance FiducialMath::solve_t_map_camera(const Observations &observations,
                                                           const std::vector<TransformWithCovariance> &t_map_markers,
                                                           double marker_length)
  {
    auto cv_t_map_camera = cv_->solve_t_map_camera(observations, t_map_markers, marker_length);
    auto sam_t_map_camera = sam_->solve_t_map_camera(observations, t_map_markers, marker_length);
    return sam_t_map_camera;
  }

  Observations FiducialMath::detect_markers(std::shared_ptr<cv_bridge::CvImage> &color,
                                            std::shared_ptr<cv_bridge::CvImage> &color_marked)
  {
    return cv_->detect_markers(color, color_marked);
  }

  void FiducialMath::annotate_image_with_marker_axis(std::shared_ptr<cv_bridge::CvImage> &color_marked,
                                                     const TransformWithCovariance &t_camera_marker)
  {
    cv_->annotate_image_with_marker_axis(color_marked, t_camera_marker);
  }

  void FiducialMath::update_map(const TransformWithCovariance &t_map_camera,
                                const Observations &observations,
                                Map &map)
  {
    cv_->update_map(t_map_camera, observations, map);
  }

}
