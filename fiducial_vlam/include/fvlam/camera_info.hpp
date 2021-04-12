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
// Camera class
// ==============================================================================

  class Camera
  {
  private:
    // The id of the camera
    std::uint64_t id_;

    std::string imager_frame_id_;

    // The pose of the camera in some world frame. Which frame is world depends on the context.
    Transform3WithCovariance t_map_camera_;

  public:
    Camera() = delete;

    Camera(std::uint64_t id, std::string imager_frame_id, Transform3WithCovariance t_map_camera) :
      id_(id), imager_frame_id_{std::move(imager_frame_id)}, t_map_camera_(std::move(t_map_camera))
    {}

    auto id() const
    { return id_; }

    const auto &t_map_camera() const
    { return t_map_camera_; }

    const auto &imager_frame_id() const
    { return imager_frame_id_; }

    template<class T>
    static Camera from(T &other);

    template<class T>
    T to() const;

    template<class T>
    void to(T &other) const;

    bool equals(const Camera &other, double tol = 1.0e-9, bool check_relative_also = true) const;
  };

// ==============================================================================
// CameraInfo class
// ==============================================================================

// This should really be called an ImagerInfo structure. But in the case where there
// is only one imager calling it a CameraInfo is equivalent. Cases that have multiple
// imagers will have to remember that this structure is named incorrectly.
  class CameraInfo
  {
  public:
    using CameraMatrix = Eigen::Matrix3d;
    using DistCoeffs = Eigen::Matrix<double, 5, 1>; // k1, k2, p1, p2 [, k3 [, k4, k5, k6]]

  private:
    std::string imager_frame_id_;
    std::uint32_t width_;
    std::uint32_t height_;
    CameraMatrix camera_matrix_;
    DistCoeffs dist_coeffs_;
    Transform3 t_camera_imager_;

  public:
    CameraInfo() :
      imager_frame_id_{},
      width_{0}, height_{0},
      camera_matrix_{CameraMatrix::Identity()},
      dist_coeffs_{DistCoeffs::Zero()},
      t_camera_imager_{}
    {}

    CameraInfo(std::uint32_t width, std::uint32_t height, CameraMatrix camera_matrix, DistCoeffs dist_coeffs) :
      imager_frame_id_{},
      width_{width}, height_{height},
      camera_matrix_{std::move(camera_matrix)},
      dist_coeffs_{std::move(dist_coeffs)},
      t_camera_imager_{}
    {}

    CameraInfo(double fx, double fy, double s, double u0, double v0) :
      imager_frame_id_{},
      width_{std::uint32_t(std::ceil(u0 * 2))}, height_{std::uint32_t(std::ceil(v0 * 2))},
      camera_matrix_{(CameraMatrix() << fx, s, u0, 0.0, fy, v0, 0.0, 0.0, 1.0).finished()},
      dist_coeffs_{DistCoeffs::Zero()},
      t_camera_imager_{}
    {}

    CameraInfo(double fx, double fy, double s, double u0, double v0,
               double k1, double k2, double p1, double p2, double k3) :
      imager_frame_id_{},
      width_{std::uint32_t(std::ceil(u0 * 2))}, height_{std::uint32_t(std::ceil(v0 * 2))},
      camera_matrix_{(CameraMatrix() << fx, s, u0, 0.0, fy, v0, 0.0, 0.0, 1.0).finished()},
      dist_coeffs_{(DistCoeffs() << k1, k2, p1, p2, k3).finished()},
      t_camera_imager_{}
    {}

    CameraInfo(std::string imager_frame_id,
               std::uint32_t width, std::uint32_t height,
               CameraMatrix camera_matrix, DistCoeffs dist_coeffs,
               Transform3 t_camera_imager) :
      imager_frame_id_{std::move(imager_frame_id)},
      width_{width}, height_{height},
      camera_matrix_{std::move(camera_matrix)},
      dist_coeffs_{std::move(dist_coeffs)},
      t_camera_imager_{std::move(t_camera_imager)}
    {}

    CameraInfo(std::string imager_frame_id,
               const CameraInfo &base,
               Transform3 t_camera_imager) :
      imager_frame_id_{std::move(imager_frame_id)},
      width_{base.width_}, height_{base.height_},
      camera_matrix_{base.camera_matrix_},
      dist_coeffs_{base.dist_coeffs_},
      t_camera_imager_{std::move(t_camera_imager)}
    {}

    auto &imager_frame_id() const
    { return imager_frame_id_; }

    auto &width() const
    { return width_; }

    auto &height() const
    { return height_; }

    const auto &camera_matrix() const
    { return camera_matrix_; }

    const auto &dist_coeffs() const
    { return dist_coeffs_; }

    const auto &t_camera_imager() const
    { return t_camera_imager_; }

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

  class CameraInfoMap
  {
    std::map<std::string, CameraInfo> m_{};

  public:
    auto &m_mutable()
    { return m_; }

    auto &m() const
    { return m_; }

    auto size() const
    { return m_.size(); }

    auto empty() const
    { return m_.empty(); }

    auto &first() const
    {
      assert(!m_.empty());
      return m_.begin()->second;
    }

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
