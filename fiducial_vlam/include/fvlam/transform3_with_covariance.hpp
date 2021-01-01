#pragma once
#pragma ide diagnostic ignored "modernize-use-nodiscard"
#pragma ide diagnostic ignored "NotImplementedFunctions"
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"
#pragma ide diagnostic ignored "OCUnusedStructInspection"
#pragma ide diagnostic ignored "OCUnusedTypeAliasInspection"

#include <Eigen/Geometry>

namespace fvlam
{

// ==============================================================================
// Translate2 class
// ==============================================================================

  class Translate2
  {
  public:
    using MuVector = Eigen::Vector2d;
    using TangentVector = Eigen::Matrix<double, MuVector::MaxSizeAtCompileTime, 1>;
    using CovarianceMatrix = Eigen::Matrix<double, MuVector::MaxSizeAtCompileTime, MuVector::MaxSizeAtCompileTime>;

  private:
    MuVector t_;

  public:
    Translate2() :
      t_{MuVector::Zero()}
    {}

    explicit Translate2(MuVector t) :
      t_(std::move(t))
    {}

    Translate2(double x, double y) :
      t_(x, y)
    {}

    const auto &x() const
    { return t_.x(); } //
    const auto &y() const
    { return t_.y(); } //

    const auto &t() const
    { return t_; }

    MuVector mu() const
    { return t_; }

    template<class T>
    static Translate2 from(const T &other);

    template<class T>
    T to() const;

    std::string to_string() const;

    Translate2 operator+(const Translate2 &other) const
    {
      return Translate2(t_ + other.t_);
    }

    Translate2 operator*(double factor) const
    {
      return Translate2(t_ * factor);
    }
  };

// ==============================================================================
// Translate3 class
// ==============================================================================

  class Translate3
  {
  public:
    using MuVector = Eigen::Vector3d;
    using TangentVector = Eigen::Matrix<double, MuVector::MaxSizeAtCompileTime, 1>;
    using CovarianceMatrix = Eigen::Matrix<double, MuVector::MaxSizeAtCompileTime, MuVector::MaxSizeAtCompileTime>;

  private:
    MuVector t_;

  public:
    Translate3() :
      t_{MuVector::Zero()}
    {}

    explicit Translate3(MuVector t) :
      t_(std::move(t))
    {}

    Translate3(double x, double y, double z) :
      t_(x, y, z)
    {}

    const auto &x() const
    { return t_.x(); } //
    const auto &y() const
    { return t_.y(); } //
    const auto &z() const
    { return t_.z(); } //

    const auto &t() const
    { return t_; }

    MuVector mu() const
    { return t_; }

    template<class T>
    static Translate3 from(const T &other);

    template<class T>
    T to() const;

    std::string to_string() const; //
    static std::string to_cov_string(const CovarianceMatrix &cov); //

    /// Exponential map at identity - create a translation from canonical coordinates \f$ [T_x,T_y,T_z] \f$
    static Translate3 Expmap(const TangentVector &x)
    { return Translate3(x); }

    /// Log map at identity - return the canonical coordinates \f$ [T_x,T_y,T_z] \f$ of this translation
    static TangentVector Logmap(const Translate3 &translate3)
    { return translate3.t_; }

    Translate3 cross(const Translate3 &v) const
    { return Translate3{t_.cross(v.t_)}; }

    Translate3 operator+(const Translate3 &other) const
    {
      return Translate3(t_ + other.t_);
    }

    Translate3 operator*(double factor) const
    {
      return Translate3(t_ * factor);
    }
  };

// ==============================================================================
// Translate3WithCovariance class
// ==============================================================================

  class Translate3WithCovariance
  {
  public:
    using MuVector = Eigen::Matrix<double, Translate3::MuVector::MaxSizeAtCompileTime +
                                           Translate3::CovarianceMatrix::MaxSizeAtCompileTime, 1>;

  private:
    bool is_valid_;
    bool is_cov_valid_;
    Translate3 t_;
    Translate3::CovarianceMatrix cov_;

  public:
    Translate3WithCovariance() :
      is_valid_{false}, is_cov_valid_{false}, t_{}, cov_{Translate3::CovarianceMatrix::Zero()}
    {}

    explicit Translate3WithCovariance(Translate3 t) :
      is_valid_{true}, is_cov_valid_{false}, t_{std::move(t)}, cov_{Translate3::CovarianceMatrix::Zero()}
    {}

    Translate3WithCovariance(Translate3 t, Translate3::CovarianceMatrix cov) :
      is_valid_{true}, is_cov_valid_{true}, t_{std::move(t)}, cov_{std::move(cov)}
    {}

    auto is_valid() const
    { return is_valid_; }

    auto is_cov_valid() const
    { return is_cov_valid_; }

    const auto &t() const
    { return t_; }

    const auto &cov() const
    { return cov_; }

    template<class T>
    static Translate3WithCovariance from(const T &other);

    template<class T>
    T to() const;

    std::string to_string() const;
  };

// ==============================================================================
// Rotate3 class
// ==============================================================================

  class Rotate3
  {
  public:
    using MuVector = Eigen::Matrix<double, 3, 1>;
    using TangentVector = Eigen::Matrix<double, 3, 1>;
    using RotationMatrix = Eigen::Matrix<double, 3, 3>;
    using CovarianceMatrix = Eigen::Matrix<double, MuVector::MaxSizeAtCompileTime, MuVector::MaxSizeAtCompileTime>;

  private:
    using Derived = Eigen::Quaterniond;
    Derived q_{Derived::Identity()};
    RotationMatrix debug_r_{RotationMatrix::Zero()};

  public:
    Rotate3() = default;

    explicit Rotate3(const Derived &q) :
      q_(q), debug_r_{q_.toRotationMatrix()}
    {}

    explicit Rotate3(const RotationMatrix &rotation_matrix) :
      q_(rotation_matrix), debug_r_{q_.toRotationMatrix()}
    {}

    static Rotate3 Rx(double x)
    { return Rotate3{Derived{Eigen::AngleAxisd{x, Eigen::Vector3d::UnitX()}}}; }

    static Rotate3 Ry(double y)
    { return Rotate3{Derived{Eigen::AngleAxisd{y, Eigen::Vector3d::UnitY()}}}; }

    static Rotate3 Rz(double z)
    { return Rotate3{Derived{Eigen::AngleAxisd{z, Eigen::Vector3d::UnitZ()}}}; }

    static Rotate3 RzRyRx(double x, double y, double z)
    {
      return Rotate3{Derived{Eigen::AngleAxisd{z, Eigen::Vector3d::UnitZ()}} *
                     Derived{Eigen::AngleAxisd{y, Eigen::Vector3d::UnitY()}} *
                     Derived{Eigen::AngleAxisd{x, Eigen::Vector3d::UnitX()}}};
    }

    static Rotate3 Ypr(double y, double p, double r)
    { return RzRyRx(r, p, y); }

    const auto &q() const
    { return q_; }

    RotationMatrix rotation_matrix() const
    { return q_.toRotationMatrix(); }

    MuVector xyz() const;

    MuVector mu() const
    { return xyz(); }

    template<class T>
    static Rotate3 from(const T &other);

    template<class T>
    T to() const;

    std::string to_string() const;

    Rotate3 inverse() const
    {
      return Rotate3(q_.inverse());
    }

    /// Exponential map at identity - create a rotation from canonical coordinates \f$ [R_x,R_y,R_z] \f$
    static Rotate3 Expmap(const TangentVector &x);

    /// Log map at identity - return the canonical coordinates \f$ [R_x,R_y,R_z] \f$ of this rotation
    static TangentVector Logmap(const Rotate3 &rotate3);

    struct ChartAtOrigin
    {
      static Rotate3 retract(const TangentVector &v)
      { return Expmap(v); } //
      static TangentVector local(const Rotate3 &r)
      { return Logmap(r); } //
    };

    Rotate3 compose(const Rotate3 &other) const
    { return *this * other; } //
    Rotate3 between(const Rotate3 &other) const
    { return (*this).inverse() * other; } //

    Rotate3 retract(const TangentVector &v) const
    { return compose(ChartAtOrigin::retract(v)); } //
    TangentVector local_coordinates(const Rotate3 &other) const
    { return ChartAtOrigin::local(between(other)); } //

    Rotate3 operator*(const Rotate3 &other) const
    {
      return Rotate3(q_ * other.q_);
    }

    Translate3 operator*(const Translate3 &other) const
    {
      return Translate3(q_ * other.t());
    }
  };

// ==============================================================================
// Transform3 class
// ==============================================================================

  class Transform3
  {
  public:
    using MuVector = Eigen::Matrix<double,
      Rotate3::MuVector::MaxSizeAtCompileTime +
      Translate3::MuVector::MaxSizeAtCompileTime, 1>;
    using TangentVector = Eigen::Matrix<double,
      Rotate3::MuVector::MaxSizeAtCompileTime +
      Translate3::MuVector::MaxSizeAtCompileTime, 1>;
    using CovarianceMatrix = Eigen::Matrix<double, MuVector::MaxSizeAtCompileTime, MuVector::MaxSizeAtCompileTime>;

  private:
    std::uint64_t id_;
    Rotate3 r_;
    Translate3 t_;

  public:
    Transform3() :
      id_{0}, r_{}, t_{}
    {}

    Transform3(Rotate3 r, Translate3 t) :
      id_{0}, r_(std::move(r)), t_(std::move(t))
    {}

    Transform3(std::uint64_t id, Rotate3 r, Translate3 t) :
      id_{id}, r_(std::move(r)), t_(std::move(t))
    {}

    Transform3(std::uint64_t id, Transform3 tf) :
      id_{id}, r_(std::move(tf.r_)), t_(std::move(tf.t_))
    {}

    Transform3(double rx, double ry, double rz, double tx, double ty, double tz) :
      id_{0}, r_{Rotate3::RzRyRx(rx, ry, rz)}, t_(Translate3{tx, ty, tz})
    {}

    explicit Transform3(const MuVector &mu) :
      id_{0},
      r_(Rotate3::RzRyRx(mu(0), mu(1), mu(2))),
      t_(Translate3(mu(3), mu(4), mu(5)))
    {}

    Transform3(std::uint64_t id, const MuVector &mu) :
      id_{id},
      r_(Rotate3::RzRyRx(mu(0), mu(1), mu(2))),
      t_(Translate3(mu(3), mu(4), mu(5)))
    {}

    auto id() const
    { return id_; }

    const auto &r() const
    { return r_; }

    const auto &t() const
    { return t_; }

    MuVector mu() const
    { return (MuVector() << r_.mu(), t_.mu()).finished(); }

    template<class T>
    static Transform3 from(const T &other);

    template<class T>
    T to() const;

    template<class T>
    static CovarianceMatrix cov_from(const T &other);

    template<class T>
    static T cov_to(const CovarianceMatrix &cov);

    std::string to_string(bool also_id = false) const; //
    static std::string to_cov_string(const CovarianceMatrix &cov); //

    Transform3 inverse() const
    {
      auto qi = r_.q().inverse();
      return Transform3(Rotate3(qi), Translate3(qi * -t_.t()));
    }

    /// Exponential map at identity - create a transform from canonical coordinates \f$ [R_x,R_y,R_z,T_x,T_y,T_z] \f$
    static Transform3 Expmap(const TangentVector &x);

    /// Log map at identity - return the canonical coordinates \f$ [R_x,R_y,R_z,T_x,T_y,T_z] \f$ of this transform
    static TangentVector Logmap(const Transform3 &transform3);

    struct ChartAtOrigin
    {
      static Transform3 retract(const TangentVector &v); //
      static TangentVector local(const Transform3 &pose); //
    };

    Transform3 compose(const Transform3 &other) const
    { return *this * other; } //
    Transform3 between(const Transform3 &other) const
    { return (*this).inverse() * other; } //

    Transform3 retract(const TangentVector &v) const
    { return compose(ChartAtOrigin::retract(v)); } //
    TangentVector local_coordinates(const Transform3 &other) const
    { return ChartAtOrigin::local(between(other)); } //

    Translate3 operator*(const Translate3 &other) const
    {
      return Translate3(r_ * other + t_);
    }

    Transform3 operator*(const Transform3 &other) const
    {
      return Transform3{r_ * other.r_, t_ + r_ * other.t_};
    }
  };

// ==============================================================================
// Transform3WithCovariance class
// ==============================================================================

  class Transform3WithCovariance
  {
  public:
    using MuVector = Eigen::Matrix<double, Transform3::MuVector::MaxSizeAtCompileTime +
                                           Transform3::CovarianceMatrix::MaxSizeAtCompileTime, 1>;

  private:
    bool is_valid_;
    bool is_cov_valid_;
    Transform3 tf_;
    Transform3::CovarianceMatrix cov_;

  public:
    Transform3WithCovariance() :
      is_valid_{false}, is_cov_valid_{false}, tf_{}, cov_{Transform3::CovarianceMatrix::Zero()}
    {}

    explicit Transform3WithCovariance(Transform3 tf) :
      is_valid_(true), is_cov_valid_{false}, tf_(std::move(tf)), cov_(Transform3::CovarianceMatrix::Zero())
    {}

    Transform3WithCovariance(Transform3 tf, Transform3::CovarianceMatrix cov) :
      is_valid_(true), is_cov_valid_{true}, tf_(std::move(tf)), cov_(std::move(cov))
    {}

    auto is_valid() const
    { return is_valid_; }

    auto is_cov_valid() const
    { return is_cov_valid_; }

    const auto &tf() const
    { return tf_; }

    const auto &cov() const
    { return cov_; }

    auto id() const
    { return tf_.id(); }

    template<class T>
    static Transform3WithCovariance from(const T &other);

    template<class T>
    T to() const;

    std::string to_string() const;
  };
}

