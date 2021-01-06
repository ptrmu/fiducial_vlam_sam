
#include "fiducial_vlam_msgs/msg/map.hpp"
#include "fiducial_vlam_msgs/msg/observations.hpp"
#include "fvlam/camera_info.hpp"
#include "fvlam/marker.hpp"
#include "fvlam/observation.hpp"
#include "fvlam/transform3_with_covariance.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace fvlam
{

// ==============================================================================
// from fvlam/transform3_with_covariance.hpp
// ==============================================================================

  template<>
  Transform3 Transform3::from<tf2::Transform>(tf2::Transform &other)
  {
    auto q = other.getRotation();
    auto t = other.getOrigin();
    return Transform3{Rotate3{Rotate3::Derived{q.w(), q.x(), q.y(), q.z()}}, Translate3{t.x(), t.y(), t.x()}};
  }

  template<>
  tf2::Transform Transform3::to<tf2::Transform>() const
  {
    tf2::Quaternion q{r_.q().x(), r_.q().y(), r_.q().z(), r_.q().w()};
    tf2::Vector3 t{t_.x(), t_.y(), t_.z()};
    return tf2::Transform{q, t};
  }

  template<>
  geometry_msgs::msg::Transform Transform3::to<geometry_msgs::msg::Transform>() const
  {
    return tf2::toMsg(to<tf2::Transform>());
  }

  template<>
  Transform3 Transform3::from<geometry_msgs::msg::Pose>(geometry_msgs::msg::Pose &other)
  {
    tf2::Transform tf;
    tf2::fromMsg(other, tf);
    return Transform3::from(tf);
  }

  template<>
  geometry_msgs::msg::Pose Transform3::to<geometry_msgs::msg::Pose>() const
  {
    geometry_msgs::msg::Pose pose;
    tf2::toMsg(to<tf2::Transform>(), pose);
    return pose;
  }

  template<>
  geometry_msgs::msg::PoseWithCovariance::_covariance_type Transform3::cov_to
    <geometry_msgs::msg::PoseWithCovariance::_covariance_type>(const CovarianceMatrix &cov)
  {
    geometry_msgs::msg::PoseWithCovariance::_covariance_type new_cov;
    for (int r = 0; r < 6; r += 1) {
      for (int c = 0; c < 6; c += 1) {
        new_cov[r * 6 + c] = cov(r, c);
      }
    }
    return new_cov;
  }

  template<>
  geometry_msgs::msg::PoseWithCovariance Transform3WithCovariance::to<geometry_msgs::msg::PoseWithCovariance>() const
  {
    geometry_msgs::msg::PoseWithCovariance msg;
    msg.pose = tf_.to<geometry_msgs::msg::Pose>();
    msg.covariance = Transform3::cov_to<geometry_msgs::msg::PoseWithCovariance::_covariance_type>(cov());
    return msg;
  }

// ==============================================================================
// from fvlam/camera_info.hpp
// ==============================================================================

  template<>
  CameraInfo CameraInfo::from<fiducial_vlam_msgs::msg::Observations>(fiducial_vlam_msgs::msg::Observations &other)
  {
    auto &m = other.camera_info;
    return CameraInfo{m.k[0], m.k[4],
                      0,
                      m.k[2], m.k[5],
                      m.d[0], m.d[1],
                      m.d[2], m.d[3],
                      m.d[4]};
  }

// ==============================================================================
// from fvlam/marker_map.hpp
// ==============================================================================

  template<>
  void Marker::to<visualization_msgs::msg::Marker>(visualization_msgs::msg::Marker &other) const
  {
    other.id = id_;
    other.pose = t_world_marker().tf().to<geometry_msgs::msg::Pose>();
    other.type = visualization_msgs::msg::Marker::CUBE;
    other.action = visualization_msgs::msg::Marker::ADD;
    other.scale.x = 0.1;
    other.scale.y = 0.1;
    other.scale.z = 0.01;
    other.color.r = 1.f;
    other.color.g = 1.f;
    other.color.b = 0.f;
    other.color.a = 1.f;
  }

  template<>
  void MarkerMap::to<fiducial_vlam_msgs::msg::Map>(fiducial_vlam_msgs::msg::Map &other) const
  {
    auto &msg = other;
    for (auto &id_marker_pair : markers_) {
      auto &marker = id_marker_pair.second;
      msg.ids.emplace_back(marker.id());
      msg.poses.emplace_back(marker.t_world_marker().to<geometry_msgs::msg::PoseWithCovariance>());
      msg.fixed_flags.emplace_back(marker.is_fixed() ? 1 : 0);
    }
    msg.marker_length = marker_length_;
    msg.map_style = 0;
  }

// ==============================================================================
// from fvlam/observation.hpp
// ==============================================================================

  template<>
  Observations Observations::from<fiducial_vlam_msgs::msg::Observations>(
    fiducial_vlam_msgs::msg::Observations &other)
  {
    fvlam::Observations observations;
    for (auto &obs : other.observations) {
      observations.add(Observation(obs.id,
                                   obs.x0, obs.y0,
                                   obs.x1, obs.y1,
                                   obs.x2, obs.y2,
                                   obs.x3, obs.y3));
    }
    return observations;
  }
}