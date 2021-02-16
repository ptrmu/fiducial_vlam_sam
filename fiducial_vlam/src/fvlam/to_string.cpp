
#include <string>

#include "fvlam/camera_info.hpp"
#include "fvlam/logger.hpp"
#include "fvlam/marker.hpp"
#include "fvlam/observation.hpp"
#include "fvlam/observations_bundle.hpp"
#include "fvlam/transform3_with_covariance.hpp"

namespace fvlam
{
  class NumFmt
  {
    int width_;
    int precision_;
    double val_{};
    bool set_{false};

  public:
    NumFmt(int width, int precision)
      : width_(width), precision_(precision)
    {
    }

    NumFmt &operator()(double val)
    {
      set_ = true;
      val_ = val;
      return *this;
    }

    friend std::ostream &
    operator<<(std::ostream &dest, NumFmt &fmt)
    {
//      dest.setf(std::ios_base::fixed, std::ios_base::floatfield);
      dest.unsetf(std::ios_base::floatfield);
      dest.precision(fmt.precision_);
      dest.width(fmt.width_);
      if (fmt.set_) {
        fmt.set_ = false;
        dest << (abs(fmt.val_) < 1.e-6 ? 0.0 : fmt.val_);
      }
      return dest;
    }
  };

  // Return a vector as a string containing a row of numbers
  template<typename FixedSizeVector>
  static std::string to_row_str(const FixedSizeVector &v)
  {
    std::stringstream ss{};
    NumFmt nf(9, 3);
    for (int r = 0; r < FixedSizeVector::MaxSizeAtCompileTime; r += 1) {
      if (r != 0) {
        ss << " ";
      }
      ss << nf(v(r));
    }
    return ss.str();
  }

  template<class FixedSizeMatrix>
  static std::string to_matrix_str(const FixedSizeMatrix &m, bool lower_only)
  {
    std::stringstream ss{};
    NumFmt nf(9, 3);
    for (int r = 0; r < m.rows(); r += 1) {
      if (r != 0) {
        ss << std::endl;
      }
      for (int c = 0; c <= (lower_only ? r : m.cols()); c += 1) {
        if (c != 0) {
          ss << " ";
        }
        ss << nf(m(r, c));
      }
    }
    return ss.str();
  }

  static std::string to_id_str(const std::uint64_t id)
  {
    std::stringstream ss{};
    ss.width(12);
    ss << id;
    return ss.str();
  }

// ==============================================================================
// from fvlam/camera_info.hpp
// ==============================================================================

// ==============================================================================
// from fvlam/marker.hpp
// ==============================================================================

  std::string Marker::to_string(bool also_cov) const
  {
    std::stringstream ss{};

    NumFmt nf(9, 3);

    if (also_cov) {
      ss << nf(id_) << std::endl << t_map_marker_.to_string();
    } else {
      ss << nf(id_) << t_map_marker_.tf().to_string();
    }
    return ss.str();
  }

  std::string Marker::to_id_string() const
  {
    return to_id_str(id_);
  }

  std::string Marker::to_corners_f_world_string(double marker_length) const
  {
    std::stringstream ss{};
    NumFmt nf(9, 3);

    auto corners_f_world = calc_corners3_f_world(marker_length);
    for (std::size_t c = 0; c < ArraySize; c += 1) {
      if (c != 0) {
        ss << " ";
      }
      ss << "(";
      for (int r = 0; r < Marker::Element3::MuVector::MaxRowsAtCompileTime; r += 1) {
        if (r != 0) {
          ss << " ";
        }
        ss << nf(corners_f_world[c].t()(r));
      }
      ss << ")";
    }

    return ss.str();
  }

  std::string MarkerMap::to_string(bool also_cov) const
  {
    std::stringstream ss{};
    bool first{true};

    // Assume a map keeps items sorted
    for (auto &kvp : *this) {
      ss << (first ? "" : "\n") << "marker id:" << kvp.second.to_string(also_cov);
      first = false;
    }

    return ss.str();
  }

// ==============================================================================
// from fvlam/observation.hpp
// ==============================================================================

  std::string Observation::to_string() const
  {
    std::stringstream ss{};
    NumFmt nf(9, 3);

    ss << to_id_str(id_) << "  ";

    for (std::size_t c = 0; c < Observation::ArraySize; c += 1) {
      if (c != 0) {
        ss << " ";
      }
      ss << "(" << nf(corners_f_image_[c].x()) << "," << nf(corners_f_image_[c].y()) << ")";
    }

    return ss.str();
  }

// ==============================================================================
// from fvlam/observations_bundle.hpp
// ==============================================================================

  std::string ObservationsBundles::to_string(bool also_cov) const
  {
    (void) also_cov;
    std::stringstream ss{};
    NumFmt nf(9, 3);

//    ss << to_id_str(id_) << "  ";
//
//    for (std::size_t c = 0; c < Observation::ArraySize; c += 1) {
//      if (c != 0) {
//        ss << " ";
//      }
//      ss << "(" << nf(corners_f_image_[c].x()) << "," << nf(corners_f_image_[c].y()) << ")";
//    }

    return ss.str();
  }

// ==============================================================================
// from fvlam/transform3_with_covariance.hpp
// ==============================================================================

  std::string Translate2::to_string() const
  {
    return to_row_str(t_);
  }

  std::string Translate3::to_string() const
  {
    return to_row_str(t_);
  }

  std::string Translate3::cov_to_string(const Translate3::CovarianceMatrix &cov)
  {
    return to_matrix_str(cov, true);
  }

  std::string Translate3WithCovariance::to_string() const
  {
    return t_.to_string() + "\n" + Translate3::cov_to_string(cov_);
  }

  std::string Rotate3::to_string() const
  {
    return to_row_str(xyz());
  }

  std::string Transform3::to_string() const
  {
    return r_.to_string() + " " + t_.to_string();
  }

  std::string Transform3::cov_to_string(const Transform3::CovarianceMatrix &cov)
  {
    return to_matrix_str(cov, true);
  }

  std::string Transform3WithCovariance::to_string() const
  {
    return tf_.to_string() + "\n" + Transform3::cov_to_string(cov_);
  }


// ==============================================================================
// from fvlam/Transform3_with_covariance.hpp
// ==============================================================================

// ==============================================================================
// equals methods
// ==============================================================================

  static bool test_double(double a, double b, double tol, bool check_relative_also)
  {
    using std::abs;
    using std::isnan;
    using std::isinf;

    double DOUBLE_MIN_NORMAL = std::numeric_limits<double>::min() + 1.0;
    double larger = (abs(b) > abs(a)) ? abs(b) : abs(a);

    // handle NaNs
    if (isnan(a) || isnan(b)) {
      return isnan(a) && isnan(b);
    }
      // handle inf
    else if (isinf(a) || isinf(b)) {
      return isinf(a) && isinf(b);
    }
      // If the two values are zero or both are extremely close to it
      // relative error is less meaningful here
    else if (a == 0 || b == 0 || (abs(a) + abs(b)) < DOUBLE_MIN_NORMAL) {
      return abs(a - b) <= tol * DOUBLE_MIN_NORMAL;
    }
      // Check if the numbers are really close.
      // Needed when comparing numbers near zero or tol is in vicinity.
    else if (abs(a - b) <= tol) {
      return true;
    }
      // Check for relative error
    else if (abs(a - b) <=
             tol * std::min(larger, std::numeric_limits<double>::max()) &&
             check_relative_also) {
      return true;
    }

    return false;
  }

  template<class T>
  static bool test_fixed_matrices(const T &a, const T &b, double tol = 1.0e-9, bool check_relative_also = true)
  {
    for (int r = 0; r < T::MaxSizeAtCompileTime; r += 1) {
      if (!test_double(a(r), b(r), tol, check_relative_also)) {
        return false;
      }
    }
    return true;
  }

  template<class T>
  static bool equals_vector(const std::vector<T> &a, const std::vector<T> &b,
                            double tol, bool check_relative_also)
  {
    if (a.size() != b.size()) {
      return false;
    }
    for (std::size_t i = 0; i < a.size(); i += 1) {
      if (!a[i].equals(b[i], tol, check_relative_also)) {
        return false;
      }
    }
    return true;
  }

  template<typename K, class V>
  static bool equals_map(const std::map<K, V> &a, const std::map<K, V> &b,
                         double tol, bool check_relative_also)
  {
    if (a.size() != b.size()) {
      return false;
    }
    for (auto &kvp : a) {
      auto it = b.find(kvp.first);
      if (it == b.end()) {
        return false;
      }
      if (!kvp.second.equals((*it).second, tol, check_relative_also)) {
        return false;
      }
    }
    return true;
  }


  bool Translate2::equals(const Translate2 &other, double tol, bool check_relative_also) const
  {
    return test_fixed_matrices(t_, other.t_, tol, check_relative_also);
  }

  bool Translate2::cov_equals(const CovarianceMatrix &own, const CovarianceMatrix &other,
                              double tol, bool check_relative_also)
  {
    return test_fixed_matrices(own, other, tol, check_relative_also);
  }

  bool Translate3::equals(const Translate3 &other,
                          double tol, bool check_relative_also) const
  {
    return test_fixed_matrices(t_, other.t_, tol, check_relative_also);
  }

  bool Translate3::cov_equals(const CovarianceMatrix &own, const CovarianceMatrix &other,
                              double tol, bool check_relative_also)
  {
    return test_fixed_matrices(own, other, tol, check_relative_also);
  }

  bool Rotate3::equals(const Rotate3 &other, double tol, bool check_relative_also) const
  {
    return test_fixed_matrices(rotation_matrix(), other.rotation_matrix(), tol, check_relative_also);
  }

  bool Rotate3::cov_equals(const CovarianceMatrix &own, const CovarianceMatrix &other,
                           double tol, bool check_relative_also)
  {
    return test_fixed_matrices(own, other, tol, check_relative_also);
  }

  bool Transform3::equals(const Transform3 &other, double tol, bool check_relative_also) const
  {
    return r_.equals(other.r_, tol, check_relative_also) &&
           t_.equals(other.t_, tol, check_relative_also);
  }

  bool Transform3::cov_equals(const CovarianceMatrix &own, const CovarianceMatrix &other,
                              double tol, bool check_relative_also)
  {
    return test_fixed_matrices(own, other, tol, check_relative_also);
  }

  bool Transform3WithCovariance::equals(const Transform3WithCovariance &other,
                                        double tol, bool check_relative_also) const
  {
    return tf_.equals(other.tf_, tol, check_relative_also) &&
           Transform3::cov_equals(cov_, other.cov_, tol, check_relative_also);
  }

  bool CameraInfo::equals(const CameraInfo &other,
                          double tol, bool check_relative_also) const
  {
    return width_ == other.width_ &&
           height_ == other.height_ &&
           test_fixed_matrices(camera_matrix_, other.camera_matrix_, tol, check_relative_also) &&
           test_fixed_matrices(dist_coeffs_, other.dist_coeffs_, tol, check_relative_also);
  }

  bool Marker::equals(const Marker &other,
                      double tol, bool check_relative_also) const
  {
    return id_ == other.id_ &&
           is_fixed_ == other.is_fixed_ &&
           t_map_marker_.equals(other.t_map_marker_, tol, check_relative_also);
  }

  bool MapEnvironment::equals(const MapEnvironment &other,
                              double tol, bool check_relative_also) const
  {
    return description_ == other.description_ &&
           marker_dictionary_id_ == other.marker_dictionary_id_ &&
           test_double(marker_length_, other.marker_length_, tol, check_relative_also);
  }

  bool MarkerMap::equals(const MarkerMap &other,
                         double tol, bool check_relative_also) const
  {
    return map_environment_.equals(other.map_environment_, tol, check_relative_also) &&
           equals_map(*this, other, tol, check_relative_also);
  }

  bool Observation::equals(const Observation &other,
                           double tol, bool check_relative_also) const
  {
    return is_valid_ == other.is_valid_ &&
           id_ == other.id_ &&
           corners_f_image_[0].equals(other.corners_f_image_[0], tol, check_relative_also) &&
           corners_f_image_[1].equals(other.corners_f_image_[1], tol, check_relative_also) &&
           corners_f_image_[2].equals(other.corners_f_image_[2], tol, check_relative_also) &&
           corners_f_image_[3].equals(other.corners_f_image_[3], tol, check_relative_also) &&
           Translate2::cov_equals(cov_, other.cov_, tol, check_relative_also);
  }

  bool Stamp::equals(const Stamp &other,
                     double tol, bool check_relative_also) const
  {
    (void) tol;
    (void) check_relative_also;
    return sec_ == other.sec_ &&
           nanosec_ == other.nanosec_;
  }

  bool Observations::equals(const Observations &other,
                            double tol, bool check_relative_also) const
  {
    return imager_frame_id_ == other.imager_frame_id_ &&
           stamp_.equals(other.stamp_, tol, check_relative_also) &&
           equals_vector(*this, other, tol, check_relative_also);
  }

  bool ObservationsBundle::equals(const ObservationsBundle &other,
                                  double tol, bool check_relative_also) const
  {
    return camera_info_.equals(other.camera_info_, tol, check_relative_also) &&
           observations_.equals(other.observations_, tol, check_relative_also);

  }

  bool ObservationsBundles::equals(const ObservationsBundles &other,
                                   double tol, bool check_relative_also) const
  {
    return map_.equals(other.map_, tol, check_relative_also) &&
           equals_vector(bundles_, other.bundles_, tol, check_relative_also);

  }
}
