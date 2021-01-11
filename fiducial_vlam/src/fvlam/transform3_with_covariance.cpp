
#include "fvlam/transform3_with_covariance.hpp"

namespace fvlam
{
  Eigen::Vector3d Rotate3::xyz() const
  {
    const auto X = q_.toRotationMatrix();
    const double x = -atan2(-X(2, 1), X(2, 2));

#if 0
    const Eigen::Matrix3d Y = X * Rx(-x).q().toRotationMatrix();
    const double y = -atan2(Y(2, 0), Y(2, 2));

    const Eigen::Matrix3d Z = Y * Ry(-y).q().toRotationMatrix();
    const double z = -atan2(-Z(1, 0), Z(1, 1));
#else
    const auto qy = q_ * Rx(-x).q();
    const auto Y = qy.toRotationMatrix();
    const double y = -atan2(Y(2, 0), Y(2, 2));

    const auto qz = qy * Ry(-y).q();
    const auto Z = qz.toRotationMatrix();
    const double z = -atan2(-Z(1, 0), Z(1, 1));
#endif
    return Eigen::Vector3d(x, y, z);
  }

/**
 * skew symmetric rotation_matrix returns this:
 *   0  -wz   wy
 *  wz    0  -wx
 * -wy   wx    0
 * @param wx 3 dimensional vector
 * @param wy
 * @param wz
 * @return a 3*3 skew symmetric rotation_matrix
*/

  static inline Rotate3::RotationMatrix skewSymmetric(double wx, double wy, double wz)
  {
    return (Rotate3::RotationMatrix() << 0.0, -wz, +wy, +wz, 0.0, -wx, -wy, +wx, 0.0).finished();
  }

  template<class Derived>
  static inline Rotate3::RotationMatrix skewSymmetric(const Eigen::MatrixBase<Derived> &w)
  {
    return skewSymmetric(w(0), w(1), w(2));
  }

  Rotate3 Rotate3::Expmap(const Rotate3::TangentVector &omega)
  {
    double theta2(omega.dot(omega));
    double theta(std::sqrt(theta2));
    Eigen::Matrix3d W{(Rotate3::RotationMatrix()
      << 0.0, -omega.z(), +omega.y(),
      +omega.z(), 0.0, -omega.x(),
      -omega.y(), +omega.x(), 0.0).finished()};

    bool nearZero = (theta2 <= std::numeric_limits<double>::epsilon());
    if (nearZero) {
      return Rotate3(Rotate3::RotationMatrix::Identity() + W);
    }

    double sin_theta = std::sin(theta);
    double s2 = std::sin(theta / 2.0);
    double one_minus_cos = 2.0 * s2 * s2;  // numerically better than [1 - cos(theta)]
    Rotate3::RotationMatrix K = W / theta;
    Rotate3::RotationMatrix KK = K * K;
    return Rotate3(Rotate3::RotationMatrix::Identity() + sin_theta * K + one_minus_cos * KK);
  }

  Rotate3::TangentVector Rotate3::Logmap(const Rotate3 &rotate3)
  {
    using std::sin;
    using std::sqrt;

    // note switch to base 1
    const auto R = rotate3.rotation_matrix();
    const double &R11 = R(0, 0), R12 = R(0, 1), R13 = R(0, 2);
    const double &R21 = R(1, 0), R22 = R(1, 1), R23 = R(1, 2);
    const double &R31 = R(2, 0), R32 = R(2, 1), R33 = R(2, 2);

    // Get trace(R)
    const double tr = R.trace();

    TangentVector omega;

    // when trace == -1, i.e., when theta = +-pi, +-3pi, +-5pi, etc.
    // we do something special
    if (tr + 1.0 < 1e-10) {
      if (std::abs(R33 + 1.0) > 1e-5)
        omega = (M_PI / sqrt(2.0 + 2.0 * R33)) * MuVector(R13, R23, 1.0 + R33);
      else if (std::abs(R22 + 1.0) > 1e-5)
        omega = (M_PI / sqrt(2.0 + 2.0 * R22)) * MuVector(R12, 1.0 + R22, R32);
      else
        // if(std::abs(R.r1_.x()+1.0) > 1e-5)  This is implicit
        omega = (M_PI / sqrt(2.0 + 2.0 * R11)) * MuVector(1.0 + R11, R21, R31);
    } else {
      double magnitude;
      const double tr_3 = tr - 3.0;  // always negative
      if (tr_3 < -1e-7) {
        double theta = acos((tr - 1.0) / 2.0);
        magnitude = theta / (2.0 * sin(theta));
      } else {
        // when theta near 0, +-2pi, +-4pi, etc. (trace near 3.0)
        // use Taylor expansion: theta \approx 1/2-(t-3)/12 + O((t-3)^2)
        magnitude = 0.5 - tr_3 * tr_3 / 12.0;
      }
      omega = magnitude * MuVector(R32 - R23, R13 - R31, R21 - R12);
    }

    return omega;
  }

  Transform3 Transform3::Expmap(const Transform3::TangentVector &xi)
  {
    // get angular velocity omega and translational velocity v from twist xi
    Eigen::Vector3d omega(xi(0), xi(1), xi(2));
    Eigen::Vector3d v(xi(3), xi(4), xi(5));

    Rotate3 R = Rotate3::Expmap(omega);
    double theta2 = omega.dot(omega);
    if (theta2 > std::numeric_limits<double>::epsilon()) {
      Eigen::Vector3d t_parallel = omega * omega.dot(v);  // translation parallel to axis
      Eigen::Vector3d omega_cross_v = omega.cross(v);     // points towards axis
      Eigen::Vector3d t = (omega_cross_v - R.rotation_matrix() * omega_cross_v + t_parallel) / theta2;
      return Transform3(R, Translate3(t));
    } else {
      return Transform3(R, Translate3(v));
    }
  }

  Transform3::TangentVector Transform3::Logmap(const Transform3 &transform3)
  {
    const Rotate3::TangentVector w = Rotate3::Logmap(transform3.r());
    const Translate3::TangentVector T = Translate3::Logmap(transform3.t());
    const double t = w.norm();
    if (t < 1e-10) {
      TangentVector log;
      log << w, T;
      return log;
    } else {
      const Rotate3::RotationMatrix W = skewSymmetric(w / t);
      // Formula from Agrawal06iros, equation (14)
      // simplified with Mathematica, and multiplying in T to avoid matrix math
      const double Tan = tan(0.5 * t);
      const auto WT = W * T;
      const auto u = T - (0.5 * t) * WT + (1 - t / (2. * Tan)) * (W * WT);
      TangentVector log;
      log << w, u;
      return log;
    }
  }

  Transform3 Transform3::ChartAtOrigin::retract(const Transform3::TangentVector &v)
  {
    Rotate3 R = Rotate3::ChartAtOrigin::retract(v.head<3>());
    return Transform3(R, Translate3(v.tail<3>()));
  }

  Transform3::TangentVector Transform3::ChartAtOrigin::local(const Transform3 &pose)
  {
    auto omega = Rotate3::ChartAtOrigin::local(pose.r());
    return (Transform3::TangentVector{} << omega, pose.t().t()).finished();
  }

}
