#pragma once
#pragma ide diagnostic ignored "modernize-use-nodiscard"
#pragma ide diagnostic ignored "NotImplementedFunctions"
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"
#pragma ide diagnostic ignored "OCUnusedTypeAliasInspection"

#include <vector>

#include "transform3_with_covariance.hpp"

namespace fvlam
{
  class CameraInfo;

// ==============================================================================
// Stamp class
// ==============================================================================

// A time class modeled after the ROS2 message stamp class
  class Stamp
  {
    std::int32_t sec_;
    std::uint32_t nanosec_;

  public:
    Stamp() :
      Stamp(0, 0)
    {}

    Stamp(std::int32_t sec, std::uint32_t nanosec) :
      sec_{sec}, nanosec_{nanosec}
    {}

    auto sec() const
    { return sec_; }

    auto nanosec() const
    { return nanosec_; }

    template<class T>
    static Stamp from(T &other);

    template<class T>
    T to() const;

    template<class T>
    void to(T &other) const;

    std::string to_string() const;

    bool equals(const Stamp &other, double tol = 1.0e-9, bool check_relative_also = true) const;
  };

// ==============================================================================
// Observation class
// ==============================================================================

  class Observation
  {
  public:
    static constexpr size_t ArraySize = 4;
    using Element = Translate2;
    using Array = std::array<Element, ArraySize>;
    using MuVector = Eigen::Matrix<double, Element::MuVector::MaxRowsAtCompileTime * ArraySize, 1>;

  private:
    bool is_valid_;

    // The id of the marker that we observed.
    std::uint64_t id_;

    // The four marker corners in the image frame.
    // origin = upper left, x -> left to right, y -> top to bottom
    Array corners_f_image_;

    // The covariance for a corner point measurement. All corners have the same covariance.
    Element::CovarianceMatrix cov_;

  public:
    Observation() :
      is_valid_{false}, id_{0},
      corners_f_image_{Translate2(), Translate2(), Translate2(), Translate2()},
      cov_{Element::CovarianceMatrix::Zero()}
    {}

    explicit Observation(std::uint64_t id) :
      is_valid_{false}, id_{id},
      corners_f_image_{Translate2(), Translate2(), Translate2(), Translate2()},
      cov_{Element::CovarianceMatrix::Zero()}
    {}

    Observation(std::uint64_t id, Array corners_f_image) :
      is_valid_{true}, id_(id),
      corners_f_image_(std::move(corners_f_image)),
      cov_{Element::CovarianceMatrix::Zero()}
    {}

    Observation(std::uint64_t id, Array corners_f_image, Element::CovarianceMatrix cov) :
      is_valid_{true}, id_(id),
      corners_f_image_(std::move(corners_f_image)),
      cov_{std::move(cov)}
    {}

    Observation(std::uint64_t id,
                double x0, double y0,
                double x1, double y1,
                double x2, double y2,
                double x3, double y3) :
      is_valid_{true}, id_(id),
      corners_f_image_{Element{x0, y0}, Element{x1, y1}, Element{x2, y2}, Element{x3, y3}},
      cov_{Element::CovarianceMatrix::Zero()}
    {}

    auto is_valid() const
    { return is_valid_; }

    auto id() const
    { return id_; }

    auto &corners_f_image() const
    { return corners_f_image_; }

    auto &cov() const
    { return cov_; }

    template<class T>
    static Observation from(T &other);

    template<class T>
    static Observation from(std::uint64_t id, T &other);

    // If the type we are converting to is an array, a vector, or matrix, then the to() routines extract
    // the corners_f_image_ values and put them in the array.
    template<class T>
    T to() const;

    template<class T>
    void to(T &other) const;

    std::string to_string() const;

    bool equals(const Observation &other, double tol = 1.0e-9, bool check_relative_also = true) const;

    Transform3 solve_t_camera_marker(const CameraInfo &camera_info, double marker_length) const; //
    Transform3 solve_t_marker_camera(const CameraInfo &camera_info, double marker_length) const; //
    Transform3 solve_t_base_marker(const CameraInfo &camera_info, double marker_length) const; //
    Transform3 solve_t_marker_base(const CameraInfo &camera_info, double marker_length) const; //
  };

// ==============================================================================
// Observations class
// ==============================================================================

  class Observations
  {
    std::string imager_frame_id_; // One or more imagers are components of a camera.
    std::vector<Observation> v_;

  public:
    Observations(std::string imager_frame_id) :
      imager_frame_id_{std::move(imager_frame_id)}
    {}

    auto &imager_frame_id() const
    { return imager_frame_id_; }

    auto &v_mutable()
    { return v_; }

    auto &v() const
    { return v_; }

    auto size() const
    { return v_.size(); }

    auto empty() const
    { return v_.empty(); }

    template<typename T>
    static Observations from(T &other);

    template<typename T>
    T to() const;

    template<class T>
    void to(T &other) const;

    std::string to_string() const;

    bool equals(const Observations &other, double tol = 1.0e-9, bool check_relative_also = true) const;
  };

// ==============================================================================
// ObservationsSynced class
// ==============================================================================

  class ObservationsSynced
  {
    Stamp stamp_; // The stamp for the synced observations.
    std::string camera_frame_id_; // A camera has one of more imagers..
    std::vector<Observations> v_{};

  public:
    ObservationsSynced(Stamp stamp, std::string camera_frame_id) :
      stamp_(stamp), camera_frame_id_{std::move(camera_frame_id)}
    {}

    auto &stamp() const
    { return stamp_; }

    auto &camera_frame_id() const
    { return camera_frame_id_; }

    auto &v_mutable()
    { return v_; }

    auto &v() const
    { return v_; }

    auto size() const
    { return v_.size(); }

    auto empty() const
    { return v_.empty(); }

    template<typename T>
    static ObservationsSynced from(T &other);

    template<typename T>
    T to() const;

    template<class T>
    void to(T &other) const;

    std::string to_string() const;

    bool equals(const ObservationsSynced &other, double tol = 1.0e-9, bool check_relative_also = true) const;
  };
}
