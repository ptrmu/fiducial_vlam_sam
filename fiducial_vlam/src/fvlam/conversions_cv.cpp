
#include <memory>

#include "fvlam/camera_info.hpp"
#include "fvlam/marker.hpp"
#include "fvlam/observation.hpp"
#include "fvlam/transform3_with_covariance.hpp"
#include "opencv2/calib3d/calib3d.hpp"

namespace fvlam
{

// ==============================================================================
// from fvlam/transform3_with_covariance.hpp
// ==============================================================================

  template<>
  cv::Point2d Translate2::to<cv::Point2d>() const
  {
    return cv::Point2d{t_(0), t_(1)};
  }

  template<>
  Translate3 Translate3::from<cv::Vec3d>(cv::Vec3d &other)
  {
    return Translate3{other[0], other[1], other[2]};
  }

  template<>
  cv::Vec3d Translate3::to<cv::Vec3d>() const
  {
    return cv::Vec3d{t()(0), t()(1), t()(2)};
  }

  template<>
  Rotate3 Rotate3::from<cv::Vec3d>(cv::Vec3d &other)
  {
    cv::Mat rmat;
    cv::Rodrigues(other, rmat);
    Rotate3::RotationMatrix m;
    for (int row = 0; row < 3; row++) {
      for (int col = 0; col < 3; col++) {
        m(row, col) = rmat.at<double>(row, col);  // Row- vs. column-major order
      }
    }
    return Rotate3{m};
  }

  template<>
  cv::Vec3d Rotate3::to<cv::Vec3d>() const
  {
    auto m = rotation_matrix();
    cv::Matx33d rmat;
    for (int row = 0; row < 3; row++) {
      for (int col = 0; col < 3; col++) {
        rmat(row, col) = m(row, col);
      }
    }
    cv::Vec3d rvec;
    cv::Rodrigues(rmat, rvec);
    return rvec;
  }

// ==============================================================================
// from fvlam/camera_info.hpp
// ==============================================================================

  template<>
  CvCameraCalibration CameraInfo::to<CvCameraCalibration>() const
  {
    cv::Matx33d cm{};
    cv::Vec<double, 5> dc{};
    for (int r = 0; r < 3; r += 1)
      for (int c = 0; c < 3; c += 1) {
        cm(r, c) = camera_matrix_(r, c);
      }
    for (int r = 0; r < DistCoeffs::MaxRowsAtCompileTime; r += 1) {
      dc(r) = dist_coeffs_(r);
    }
    return CvCameraCalibration(cm, dc);
  }

// ==============================================================================
// from fvlam/marker.hpp
// ==============================================================================

  template<>
  std::vector<cv::Point3d> Marker::corners_f_marker<std::vector<cv::Point3d>>(double marker_length)
  {
    auto corners_f_marker = calc_corners3_f_marker(marker_length);
    return std::vector<cv::Point3d>{
      cv::Point3d{corners_f_marker[0].t()(0), corners_f_marker[0].t()(1), corners_f_marker[0].t()(2)},
      cv::Point3d{corners_f_marker[1].t()(0), corners_f_marker[1].t()(1), corners_f_marker[1].t()(2)},
      cv::Point3d{corners_f_marker[2].t()(0), corners_f_marker[2].t()(1), corners_f_marker[2].t()(2)},
      cv::Point3d{corners_f_marker[3].t()(0), corners_f_marker[3].t()(1), corners_f_marker[3].t()(2)}
    };
  }

  template<>
  std::vector<cv::Point3d> Marker::corners_f_world<std::vector<cv::Point3d>>(double marker_length) const
  {
    auto corners_f_world = calc_corners3_f_world(marker_length);
    return std::vector<cv::Point3d>{
      cv::Point3d{corners_f_world[0].t()(0), corners_f_world[0].t()(1), corners_f_world[0].t()(2)},
      cv::Point3d{corners_f_world[1].t()(0), corners_f_world[1].t()(1), corners_f_world[1].t()(2)},
      cv::Point3d{corners_f_world[2].t()(0), corners_f_world[2].t()(1), corners_f_world[2].t()(2)},
      cv::Point3d{corners_f_world[3].t()(0), corners_f_world[3].t()(1), corners_f_world[3].t()(2)}
    };
  }

  template<>
  void Marker::corners_f_world<std::vector<cv::Point3d>>(double marker_length, std::vector<cv::Point3d> &other) const
  {
    auto corners_f_world = calc_corners3_f_world(marker_length);
    other.emplace_back(cv::Point3d{corners_f_world[0].t()(0), corners_f_world[0].t()(1), corners_f_world[0].t()(2)});
    other.emplace_back(cv::Point3d{corners_f_world[1].t()(0), corners_f_world[1].t()(1), corners_f_world[1].t()(2)});
    other.emplace_back(cv::Point3d{corners_f_world[2].t()(0), corners_f_world[2].t()(1), corners_f_world[2].t()(2)});
    other.emplace_back(cv::Point3d{corners_f_world[3].t()(0), corners_f_world[3].t()(1), corners_f_world[3].t()(2)});
  }

// ==============================================================================
// from fvlam/observation.hpp
// ==============================================================================

  template<>
  std::vector<cv::Point2d> Observation::to<std::vector<cv::Point2d>>() const
  {
    return std::vector<cv::Point2d>{
      cv::Point2d{corners_f_image_[0].x(), corners_f_image_[0].y()},
      cv::Point2d{corners_f_image_[1].x(), corners_f_image_[1].y()},
      cv::Point2d{corners_f_image_[2].x(), corners_f_image_[2].y()},
      cv::Point2d{corners_f_image_[3].x(), corners_f_image_[3].y()}
    };
  }

  template<>
  void Observation::to<std::vector<cv::Point2d>>(std::vector<cv::Point2d> &other) const
  {
    other.emplace_back(cv::Point2d{corners_f_image_[0].x(), corners_f_image_[0].y()});
    other.emplace_back(cv::Point2d{corners_f_image_[1].x(), corners_f_image_[1].y()});
    other.emplace_back(cv::Point2d{corners_f_image_[2].x(), corners_f_image_[2].y()});
    other.emplace_back(cv::Point2d{corners_f_image_[3].x(), corners_f_image_[3].y()});
  }

  template<>
  Observation Observation::from<std::vector<cv::Point2d>>(
    std::uint64_t id, std::vector<cv::Point2d> &other)
  {
    return Observation(id, Observation::Array{
      Observation::Element{other[0].x, other[0].y},
      Observation::Element{other[1].x, other[1].y},
      Observation::Element{other[2].x, other[2].y},
      Observation::Element{other[3].x, other[3].y}
    });
  }

  template<>
  Observations Observations::from<std::pair<std::vector<int>, std::vector<std::vector<cv::Point2f>>>>(
    std::pair<std::vector<int>, std::vector<std::vector<cv::Point2f>>> &other)
  {
    auto &ids = other.first;
    auto &corners = other.second;

    Observations observations{0, ""}; // ToDo fix this
    for (size_t i = 0; i < ids.size(); i += 1) {
      observations.emplace_back(Observation(ids[i],
                                            corners[i][0].x, corners[i][0].y,
                                            corners[i][1].x, corners[i][1].y,
                                            corners[i][2].x, corners[i][2].y,
                                            corners[i][3].x, corners[i][3].y));
    }
    return observations;
  }

  Transform3 Observation::solve_t_camera_marker(const CameraInfo &camera_info, double marker_length) const
  {
    auto camera_calibration = camera_info.to<CvCameraCalibration>();

    // Build up two lists of corner points: 2D in the image frame, 3D in the marker frame.
    auto corners_f_marker{Marker::corners_f_marker<std::vector<cv::Point3d>>(marker_length)};
    auto corners_f_image{to<std::vector<cv::Point2d>>()};

    // Figure out marker pose.
    cv::Vec3d rvec, tvec;
    cv::solvePnP(corners_f_marker, corners_f_image,
                 camera_calibration.first, camera_calibration.second,
                 rvec, tvec);

    // rvec, tvec output from solvePnp "brings points from the model coordinate system to the
    // camera coordinate system". In this case the marker frame is the model coordinate system.
    // So rvec, tvec are the transformation t_camera_marker.
    return Transform3(Rotate3::from(rvec), Translate3::from(tvec));
  }

  Transform3 Observation::solve_t_marker_camera(const CameraInfo &camera_info, double marker_length) const
  {
    return solve_t_camera_marker(camera_info, marker_length).inverse();
  }

  Transform3 Observation::solve_t_base_marker(const CameraInfo &camera_info, double marker_length) const
  {
    return camera_info.t_base_camera() * solve_t_camera_marker(camera_info, marker_length);
  }

  Transform3 Observation::solve_t_marker_base(const CameraInfo &camera_info, double marker_length) const
  {
    return solve_t_base_marker(camera_info, marker_length).inverse();
  }

// ==============================================================================
// fvlam::Marker conversions that require Observation conversions so
// have to be after those conversions in the file.
// ==============================================================================

  template<>
  Marker::ProjectFunction Marker::project_t_world_marker<CvCameraCalibration>(
    const CvCameraCalibration &camera_calibration,
    const Transform3 &t_world_camera,
    double marker_length)
  {
    // gtsam and OpencCv use different frames for their camera coordinates. Our default
    // is the gtsam version os her we have to rotate the camera framw when using the
    // opencv project function
//    auto cv_t_world_camera = Transform3{Rotate3::RzRyRx(0.0, 0.0, 0.0), Translate3{}} * t_world_camera;
    return [
      camera_calibration,
      t_world_camera,
      marker_length]
      (const Marker &marker) -> Observation
    {
      // The camera coordinate system is described here:
      // https://docs.opencv.org/3.3.1/d9/d0c/group__calib3d.html
      // The rvec and tvec inputs to projectPoints are t_camera_world
      // The points input to projectPoints are in the world frame
      // The points output from projectPoints are in the 2D image frame
      auto t_camera_world = t_world_camera.inverse();
      auto camera_matrix = camera_calibration.first;
      auto dist_coeffs = camera_calibration.second;
      auto rvec = t_camera_world.r().to<cv::Vec3d>();
      auto tvec = t_camera_world.t().to<cv::Vec3d>();

      std::vector<cv::Point2d> image_points;
      auto corners_f_world = marker.corners_f_world<std::vector<cv::Point3d>>(marker_length);
      cv::projectPoints(corners_f_world, rvec, tvec, camera_matrix, dist_coeffs, image_points);
      return Observation::from<std::vector<cv::Point2d>>(marker.id(), image_points);
    };
  }
}

