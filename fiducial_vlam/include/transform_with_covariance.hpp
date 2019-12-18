#ifndef FIDUCIAL_VLAM_TRANSFORM_WITH_COVARIANCE_HPP
#define FIDUCIAL_VLAM_TRANSFORM_WITH_COVARIANCE_HPP

#include <array>

#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Transform.h"

namespace fiducial_vlam
{

  class TransformWithCovariance
  {
  public:
    using mu_type = std::array<double, 6>;
    using cov_type = std::array<double, 36>;

  private:
    bool is_valid_{false};
    tf2::Transform transform_;
    cov_type cov_{{0.}};

    static tf2::Transform to_transform(const mu_type &mu)
    {
      tf2::Quaternion q;
      q.setRPY(mu[3], mu[4], mu[5]);
      return tf2::Transform(q, tf2::Vector3(mu[0], mu[1], mu[2]));
    }

  public:
    TransformWithCovariance() = default;

    TransformWithCovariance(const tf2::Transform &transform, const cov_type &cov)
      : is_valid_(true), transform_(transform), cov_(cov)
    {}

    explicit TransformWithCovariance(const tf2::Transform &transform)
      : is_valid_(true), transform_(transform), cov_()
    {}

    TransformWithCovariance(const mu_type &mu, const cov_type &cov)
      : is_valid_(true), transform_(to_transform(mu)), cov_(cov)
    {}

    explicit TransformWithCovariance(const mu_type &mu)
      : is_valid_(true), transform_(to_transform(mu)), cov_()
    {}

    explicit TransformWithCovariance(const tf2::Quaternion &q)
      : is_valid_(true), transform_(q), cov_()
    {}

    TransformWithCovariance(const tf2::Quaternion &q, tf2::Vector3 &c)
      : is_valid_(true), transform_(q, c), cov_()
    {}

    auto is_valid() const
    { return is_valid_; }

    auto &transform() const
    { return transform_; }

    auto &cov() const
    { return cov_; }

    mu_type mu() const
    {
      double roll, pitch, yaw;
      transform_.getBasis().getRPY(roll, pitch, yaw);
      auto c = transform_.getOrigin();
      return mu_type{c[0], c[1], c[2], roll, pitch, yaw};
    }

    void update_simple_average(const TransformWithCovariance &newVal, int previous_update_count);
  };

  class Point3WithCovariance
  {
  public:
    using mu_type = std::array<double, 3>;
    using cov_type = std::array<double, 9>;

  private:
    bool is_valid_{false};
    tf2::Vector3 point_;
    cov_type cov_{{0.}};

    static tf2::Vector3 to_point3(const mu_type &mu)
    {
      return tf2::Vector3{mu[0], mu[1], mu[2]};
    }

  public:
    Point3WithCovariance() = default;

    Point3WithCovariance(const tf2::Vector3 &point, const cov_type &cov)
      : is_valid_{true}, point_{point}, cov_{cov}
    {}

    explicit Point3WithCovariance(const tf2::Vector3 &point)
      : is_valid_{true}, point_{point}, cov_{}
    {}

    Point3WithCovariance(const mu_type &mu, const cov_type &cov)
      : is_valid_{true}, point_{(to_point3(mu))}, cov_{cov}
    {}

    explicit Point3WithCovariance(const mu_type &mu)
      : is_valid_{true}, point_{(to_point3(mu))}, cov_{}
    {}

    auto is_valid() const
    { return is_valid_; }

    auto &point3() const
    { return point_; }

    auto &cov() const
    { return cov_; }

    mu_type mu() const
    {
      return mu_type{point_[0], point_[1], point_[2]};
    }
  };
}

#endif //FIDUCIAL_VLAM_TRANSFORM_WITH_COVARIANCE_HPP
