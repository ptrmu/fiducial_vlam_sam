
#ifndef CV_UTILS_TF_UTILS_HPP
#define CV_UTILS_TF_UTILS_HPP

#include "transform_with_covariance.hpp"
#include <vector>

namespace fiducial_vlam
{
// ==============================================================================
// TFConvert class
// ==============================================================================

  struct TFConvert
  {
    template<class TPoint>
    static std::vector<TPoint> corners_f_marker(double marker_length)
    {
      return std::vector<TPoint>{
        TPoint{-marker_length / 2.0, marker_length / 2.0, 0.0},
        TPoint{marker_length / 2.0, marker_length / 2.0, 0.0},
        TPoint{marker_length / 2.0, -marker_length / 2.0, 0.0},
        TPoint{-marker_length / 2.0, -marker_length / 2.0, 0.0}};
    }

    template<class TPoint>
    static TPoint to_point(const Vector3WithCovariance &v3wc)
    {
      return TPoint{v3wc.vector3().x(),
                    v3wc.vector3().y(),
                    v3wc.vector3().z()};
    }

    template<class TPoint>
    static TPoint to_point(const tf2::Vector3 &v3)
    {
      return TPoint{v3.x(),
                    v3.y(),
                    v3.z()};
    }

    template<class TPoint>
    static std::vector<TPoint> to_corner_points(const std::vector<Vector3WithCovariance> &corners)
    {
      std::vector<TPoint> corner_points;
      for (auto &corner : corners) {
        corner_points.emplace_back(to_point<TPoint>(corner));
      }
      return corner_points;
    }

    template<class TPoint>
    static void
    corners_f_map(const TransformWithCovariance &t_map_marker, double marker_length, std::vector<TPoint> &destination)
    {
      auto &t{t_map_marker.transform()};
      destination.insert(destination.end(), {
        to_point<TPoint>(t * tf2::Vector3{-marker_length / 2.0, marker_length / 2.0, 0.0}),
        to_point<TPoint>(t * tf2::Vector3{marker_length / 2.0, marker_length / 2.0, 0.0}),
        to_point<TPoint>(t * tf2::Vector3{marker_length / 2.0, -marker_length / 2.0, 0.0}),
        to_point<TPoint>(t * tf2::Vector3{-marker_length / 2.0, -marker_length / 2.0, 0.0})});
    }
  };
}
#endif //CV_UTILS_TF_UTILS_HPP
