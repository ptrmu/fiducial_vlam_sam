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
    static Translate2 from(T &other);

    template<class T>
    T to() const;

    template<class T>
    void to(T &other) const;

    template<class T>
    static CovarianceMatrix cov_from(T &other); //
    template<class T>
    static T cov_to(const CovarianceMatrix &cov); //
    template<class T>
    static void cov_to(const CovarianceMatrix &cov, T &other); //

    std::string to_string() const; //
    static std::string cov_to_string(const CovarianceMatrix &cov); //

    bool equals(const Translate2 &other, double tol = 1.0e-9, bool check_relative_also = true) const; //
    static bool cov_equals(const CovarianceMatrix &own, const CovarianceMatrix &other,
                           double tol = 1.0e-9, bool check_relative_also = true);

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
    static Translate3 from(T &other);

    template<class T>
    T to() const;

    template<class T>
    void to(T &other) const;

    template<class T>
    static CovarianceMatrix cov_from(T &other); //
    template<class T>
    static T cov_to(const CovarianceMatrix &cov); //
    template<class T>
    static void cov_to(const CovarianceMatrix &cov, T &other); //

    std::string to_string() const; //
    static std::string cov_to_string(const CovarianceMatrix &cov); //

    bool equals(const Translate3 &other, double tol = 1.0e-9, bool check_relative_also = true) const; //
    static bool cov_equals(const CovarianceMatrix &own, const CovarianceMatrix &other,
                           double tol = 1.0e-9, bool check_relative_also = true);

    /// Exponential map at identity - create a translation from canonical coordinates \f$ [T_x,T_y,T_z] \f$
    static Translate3 Expmap(const TangentVector &x)
    { return Translate3(x); }

    /// Log map at identity - return the canonical coordinates \f$ [T_x,T_y,T_z] \f$ of this translation
    static TangentVector Logmap(const Translate3 &translate3)
    { return translate3.t_; }

    Translate3 cross(const Translate3 &v) const
    {
      return Translate3{t_.cross(v.t_)};
    }

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
    static Translate3WithCovariance from(T &other);

    template<class T>
    T to() const;

    template<class T>
    void to(T &other) const;

    std::string to_string() const;

    bool equals(const Translate3WithCovariance &other, double tol = 1.0e-9, bool check_relative_also = true) const
    {
      return t_.equals(other.t_, tol, check_relative_also) &&
             Translate3::cov_equals(cov_, other.cov_, tol, check_relative_also);
    }
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
    using Derived = Eigen::Quaterniond;

  private:
    Derived q_{Derived::Identity()};
    RotationMatrix debug_r_{RotationMatrix::Zero()};
    MuVector debug_xyz_{MuVector::Zero()};

    static MuVector xyz(const Derived &q);

  public:
    Rotate3() = default;

    explicit Rotate3(const Derived &q) :
      q_(q), debug_r_{q_.toRotationMatrix()}, debug_xyz_{xyz(q_)}
    {}

    explicit Rotate3(const RotationMatrix &rotation_matrix) :
      q_(rotation_matrix), debug_r_{q_.toRotationMatrix()}, debug_xyz_{xyz(q_)}
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

    MuVector xyz() const
    { return xyz(q_); }

    MuVector mu() const
    { return xyz(q_); }

    template<class T>
    static Rotate3 from(T &other);

    template<class T>
    T to() const;

    template<class T>
    void to(T &other) const;

    template<class T>
    static CovarianceMatrix cov_from(T &other); //
    template<class T>
    static T cov_to(const CovarianceMatrix &cov); //
    template<class T>
    static void cov_to(const CovarianceMatrix &cov, T &other); //

    std::string to_string() const; //
    static std::string cov_to_string(const CovarianceMatrix &cov); //

    bool equals(const Rotate3 &other, double tol = 1.0e-9, bool check_relative_also = true) const; //
    static bool cov_equals(const CovarianceMatrix &own, const CovarianceMatrix &other,
                           double tol = 1.0e-9, bool check_relative_also = true);

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
    Rotate3 slerp(const Rotate3 &other, double t) const
    { return compose(Expmap(t * Logmap(between(other)))); } //

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
    Rotate3 r_;
    Translate3 t_;

  public:
    Transform3() :
      r_{}, t_{}
    {}

    Transform3(Rotate3 r, Translate3 t) :
      r_(std::move(r)), t_(std::move(t))
    {}

    Transform3(double rx, double ry, double rz, double tx, double ty, double tz) :
      r_{Rotate3::RzRyRx(rx, ry, rz)}, t_(Translate3{tx, ty, tz})
    {}

    explicit Transform3(const MuVector &mu) :
      r_(Rotate3::RzRyRx(mu(0), mu(1), mu(2))),
      t_(Translate3(mu(3), mu(4), mu(5)))
    {}

    const auto &r() const
    { return r_; }

    const auto &t() const
    { return t_; }

    MuVector mu() const
    { return (MuVector() << r_.mu(), t_.mu()).finished(); }

    template<class T>
    static Transform3 from(T &other);

    template<class T>
    T to() const;

    template<class T>
    void to(T &other) const;

    template<class T>
    static CovarianceMatrix cov_from(T &other); //
    template<class T>
    static T cov_to(const CovarianceMatrix &cov); //
    template<class T>
    static void cov_to(const CovarianceMatrix &cov, T &other); //

    std::string to_string() const; //
    static std::string cov_to_string(const CovarianceMatrix &cov); //

    bool equals(const Transform3 &other, double tol = 1.0e-9, bool check_relative_also = true) const; //
    static bool cov_equals(const CovarianceMatrix &own, const CovarianceMatrix &other,
                           double tol = 1.0e-9, bool check_relative_also = true);

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

    template<class T>
    static Transform3WithCovariance from(T &other);

    template<class T>
    T to() const;

    template<class T>
    void to(T &other) const;

    std::string to_string() const;

    bool equals(const Transform3WithCovariance &other, double tol = 1.0e-9, bool check_relative_also = true) const;
  };
}

