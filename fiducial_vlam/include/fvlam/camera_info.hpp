#pragma once
#pragma ide diagnostic ignored "modernize-use-nodiscard"
#pragma ide diagnostic ignored "NotImplementedFunctions"
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"

#include <Eigen/Geometry>

// Used for specializing the from/to methods on CameraInfo
namespace cv
{
  template<typename T, int M, int N>
  class Matx;
  template <typename T, int N>
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
    std::uint32_t width_;
    std::uint32_t height_;
    CameraMatrix camera_matrix_;
    DistCoeffs dist_coeffs_;

  public:
    CameraInfo() :
      width_{0}, height_{0}, camera_matrix_{CameraMatrix::Identity()}, dist_coeffs_{DistCoeffs::Zero()}
    {}

    CameraInfo(std::uint32_t width, std::uint32_t height, CameraMatrix camera_matrix, DistCoeffs dist_coeffs) :
      width_{width}, height_{height}, camera_matrix_{std::move(camera_matrix)}, dist_coeffs_{std::move(dist_coeffs)}
    {}

    CameraInfo(double fx, double fy, double s, double u0, double v0) :
      width_{std::uint32_t(std::ceil(u0 * 2))}, height_{std::uint32_t(std::ceil(v0 * 2))},
      camera_matrix_{(CameraMatrix() << fx, s, u0, 0.0, fy, v0, 0.0, 0.0, 1.0).finished()},
      dist_coeffs_{DistCoeffs::Zero()}
    {}

    CameraInfo(double fx, double fy, double s, double u0, double v0,
               double k1, double k2, double p1, double p2, double k3) :
      width_{std::uint32_t(std::ceil(u0 * 2))}, height_{std::uint32_t(std::ceil(v0 * 2))},
      camera_matrix_{(CameraMatrix() << fx, s, u0, 0.0, fy, v0, 0.0, 0.0, 1.0).finished()},
      dist_coeffs_{(DistCoeffs() << k1, k2, p1, p2, k3).finished()}
    {}

    auto &width() const
    { return width_; }

    auto &height() const
    { return height_; }

    const auto &camera_matrix() const
    { return camera_matrix_; }

    const auto &dist_coeffs() const
    { return dist_coeffs_; }

    template<class T>
    static CameraInfo from(T &other);

    template<class T>
    T to() const;

    template<class T>
    void to(T &other) const;

    std::string to_string() const;
  };

// Used for specializing the from/to methods on CameraInfo
  using CvCameraCalibration = std::pair<cv::Matx<double, 3, 3>, cv::Vec<double, 5>>;
}
