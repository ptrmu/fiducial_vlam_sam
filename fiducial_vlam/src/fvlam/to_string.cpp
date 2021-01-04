
#include <string>

#include "fvlam/marker.hpp"
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
// from fvlam/marker_map.hpp
// ==============================================================================

  std::string Marker::to_string(bool also_cov) const
  {
    std::stringstream ss{};

    NumFmt nf(9, 3);

    if (also_cov) {
      ss << nf(id_) << std::endl << t_world_marker_.to_string();
    } else {
      ss << nf(id_) << t_world_marker_.tf().to_string();
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
    for (auto it = markers_.begin(); it != markers_.end(); ++it) {
      ss << (first ? "" : "\n") << "marker id:" << it->second.to_string(also_cov);
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

  std::string Translate3::to_cov_string(const Translate3::CovarianceMatrix &cov)
  {
    return to_matrix_str(cov, true);
  }

  std::string Translate3WithCovariance::to_string() const
  {
    return t_.to_string() + "\n" + Translate3::to_cov_string(cov_);
  }

  std::string Rotate3::to_string() const
  {
    return to_row_str(xyz());
  }

  std::string Transform3::to_string(bool also_id) const
  {
    return (also_id ? to_id_str(id_) + "\n" : "") + r_.to_string() + " " + t_.to_string();
  }

  std::string Transform3::to_cov_string(const Transform3::CovarianceMatrix &cov)
  {
    return to_matrix_str(cov, true);
  }

  std::string Transform3WithCovariance::to_string() const
  {
    return tf_.to_string() + "\n" + Transform3::to_cov_string(cov_);
  }

}
