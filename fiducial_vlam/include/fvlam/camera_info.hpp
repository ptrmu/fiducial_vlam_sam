#pragma once
#pragma ide diagnostic ignored "modernize-use-nodiscard"
#pragma ide diagnostic ignored "NotImplementedFunctions"
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"

#include <map>

#include <Eigen/Geometry>
#include "transform3_with_covariance.hpp"

// Used for specializing the from/to methods on CameraInfo
namespace cv
{
  template<typename T, int M, int N>
  class Matx;

  template<typename T, int N>
  class Vec;
}

namespace fvlam
{
// ==============================================================================
// CameraInfo class
// ==============================================================================

  class CameraInfo
  {
  public:
    using CameraMatrix = Eigen::Matrix3d;
    using DistCoeffs = Eigen::Matrix<double, 5, 1>; // k1, k2, p1, p2 [, k3 [, k4, k5, k6]]

  private:
    std::string frame_id_;
    std::uint32_t width_;
    std::uint32_t height_;
    CameraMatrix camera_matrix_;
    DistCoeffs dist_coeffs_;
    Transform3 t_base_camera_;

  public:
    CameraInfo() :
      frame_id_{},
      width_{0}, height_{0},
      camera_matrix_{CameraMatrix::Identity()},
      dist_coeffs_{DistCoeffs::Zero()},
      t_base_camera_{}
    {}

    CameraInfo(std::uint32_t width, std::uint32_t height, CameraMatrix camera_matrix, DistCoeffs dist_coeffs) :
      frame_id_{},
      width_{width}, height_{height},
      camera_matrix_{std::move(camera_matrix)},
      dist_coeffs_{std::move(dist_coeffs)},
      t_base_camera_{}
    {}

    CameraInfo(double fx, double fy, double s, double u0, double v0) :
      frame_id_{},
      width_{std::uint32_t(std::ceil(u0 * 2))}, height_{std::uint32_t(std::ceil(v0 * 2))},
      camera_matrix_{(CameraMatrix() << fx, s, u0, 0.0, fy, v0, 0.0, 0.0, 1.0).finished()},
      dist_coeffs_{DistCoeffs::Zero()},
      t_base_camera_{}
    {}

    CameraInfo(double fx, double fy, double s, double u0, double v0,
               double k1, double k2, double p1, double p2, double k3) :
      frame_id_{},
      width_{std::uint32_t(std::ceil(u0 * 2))}, height_{std::uint32_t(std::ceil(v0 * 2))},
      camera_matrix_{(CameraMatrix() << fx, s, u0, 0.0, fy, v0, 0.0, 0.0, 1.0).finished()},
      dist_coeffs_{(DistCoeffs() << k1, k2, p1, p2, k3).finished()},
      t_base_camera_{}
    {}

    CameraInfo(const std::string &frame_id,
               std::uint32_t width, std::uint32_t height,
               CameraMatrix camera_matrix, DistCoeffs dist_coeffs,
               const Transform3 &t_base_camera) :
      frame_id_{frame_id},
      width_{width}, height_{height},
      camera_matrix_{std::move(camera_matrix)},
      dist_coeffs_{std::move(dist_coeffs)},
      t_base_camera_{t_base_camera}
    {}

    CameraInfo(const std::string &frame_id,
               const CameraInfo &base,
               const Transform3 &t_base_camera) :
      frame_id_{frame_id},
      width_{base.width_}, height_{base.height_},
      camera_matrix_{base.camera_matrix_},
      dist_coeffs_{base.dist_coeffs_},
      t_base_camera_{t_base_camera}
    {}

    auto &frame_id() const
    { return frame_id_; }

    auto &width() const
    { return width_; }

    auto &height() const
    { return height_; }

    const auto &camera_matrix() const
    { return camera_matrix_; }

    const auto &dist_coeffs() const
    { return dist_coeffs_; }

    const auto &t_base_camera() const
    { return t_base_camera_; }

    template<class T>
    static CameraInfo from(T &other);

    template<class T>
    T to() const;

    template<class T>
    void to(T &other) const;

    std::string to_string() const;

    bool equals(const CameraInfo &other, double tol = 1.0e-9, bool check_relative_also = true) const;
  };

// ==============================================================================
// CameraInfo class
// ==============================================================================

  class CameraInfoMap : public std::map<std::string, CameraInfo>
  {
    template<class T>
    static CameraInfoMap from(T &other);

    template<class T>
    T to() const;

    template<class T>
    void to(T &other) const;

    std::string to_string() const;

    bool equals(const CameraInfoMap &other, double tol = 1.0e-9, bool check_relative_also = true) const;
  };


// Used for specializing the from/to methods on CameraInfo
  using CvCameraCalibration = std::pair<cv::Matx<double, 3, 3>, cv::Vec<double, 5>>;
}
