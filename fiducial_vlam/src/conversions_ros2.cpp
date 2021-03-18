
#include "fiducial_vlam_msgs/msg/map.hpp"
#include "fiducial_vlam_msgs/msg/observations.hpp"
#include "fiducial_vlam_msgs/msg/observations_synced.hpp"
#include "fvlam/camera_info.hpp"
#include "fvlam/marker.hpp"
#include "fvlam/observation.hpp"
#include "fvlam/transform3_with_covariance.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
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
    return Transform3{Rotate3{Rotate3::Derived{q.w(), q.x(), q.y(), q.z()}}, Translate3{t.x(), t.y(), t.z()}};
  }

  template<>
  tf2::Transform Transform3::to<tf2::Transform>() const
  {
    tf2::Quaternion q{r_.q().x(), r_.q().y(), r_.q().z(), r_.q().w()};
    tf2::Vector3 t{t_.x(), t_.y(), t_.z()};
    return tf2::Transform{q, t};
  }

  template<>
  Transform3 Transform3::from<geometry_msgs::msg::Transform>(geometry_msgs::msg::Transform &other)
  {
    tf2::Transform tf;
    tf2::fromMsg(other, tf);
    return Transform3::from(tf);
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
  Transform3::CovarianceMatrix Transform3::cov_from<geometry_msgs::msg::PoseWithCovariance::_covariance_type>(
    geometry_msgs::msg::PoseWithCovariance::_covariance_type &other)
  {
    CovarianceMatrix cov{};
    for (int r = 0; r < CovarianceMatrix::MaxRowsAtCompileTime; r += 1) {
      for (int c = 0; c < CovarianceMatrix::MaxColsAtCompileTime; c += 1) {
        cov(r, c) = other[r * 6 + c];
      }
    }
    return cov;
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
  Transform3WithCovariance Transform3WithCovariance::from<geometry_msgs::msg::PoseWithCovariance>(
    geometry_msgs::msg::PoseWithCovariance &other)
  {
    auto tf = Transform3::from(other.pose);
    auto cov = Transform3::cov_from(other.covariance);
    return Transform3WithCovariance{std::move(tf), std::move(cov)};
  }

  template<>
  geometry_msgs::msg::PoseWithCovariance Transform3WithCovariance::to<geometry_msgs::msg::PoseWithCovariance>() const
  {
    return geometry_msgs::msg::PoseWithCovariance{}
      .set__pose(tf_.to<geometry_msgs::msg::Pose>())
      .set__covariance(Transform3::cov_to<geometry_msgs::msg::PoseWithCovariance::_covariance_type>(cov()));
  }

// ==============================================================================
// from fvlam/camera_info.hpp
// ==============================================================================

  template<>
  CameraInfo CameraInfo::from<sensor_msgs::msg::CameraInfo>(sensor_msgs::msg::CameraInfo &other)
  {
    auto &m = other;
    return CameraInfo{m.k[0], m.k[4],
                      0,
                      m.k[2], m.k[5],
                      m.d[0], m.d[1],
                      m.d[2], m.d[3],
                      m.d[4]};
  }

  template<>
  CameraInfo CameraInfo::from<fiducial_vlam_msgs::msg::CameraInfo>(
    fiducial_vlam_msgs::msg::CameraInfo &other)
  {
    auto &m = other;
    return CameraInfo{m.imager_frame_id,
                      m.width, m.height,
                      (CameraMatrix() << m.fx, 0, m.cx, 0.0, m.fy, m.cy, 0.0, 0.0, 1.0).finished(),
                      (DistCoeffs() << m.k1, m.k2, m.p1, m.p2, m.k3).finished(),
                      Transform3::from(m.t_camera_imager)};
  }

  template<>
  fiducial_vlam_msgs::msg::CameraInfo CameraInfo::to<fiducial_vlam_msgs::msg::CameraInfo>() const
  {
    return fiducial_vlam_msgs::msg::CameraInfo{}
      .set__imager_frame_id(imager_frame_id_)
      .set__width(width_)
      .set__height(height_)
      .set__fx(camera_matrix_(0, 0))
      .set__fy(camera_matrix_(1, 1))
      .set__cx(camera_matrix_(0, 2))
      .set__cy(camera_matrix_(1, 2))
      .set__k1(dist_coeffs_(0))
      .set__k2(dist_coeffs_(1))
      .set__p1(dist_coeffs_(2))
      .set__p2(dist_coeffs_(3))
      .set__k3(dist_coeffs_(4))
      .set__t_camera_imager(t_camera_imager_.to<geometry_msgs::msg::Transform>());
  }

  template<>
  CameraInfoMap CameraInfoMap::from<fiducial_vlam_msgs::msg::ObservationsSynced>(
    fiducial_vlam_msgs::msg::ObservationsSynced &other)
  {
    auto camera_info_map = CameraInfoMap{};
    for (auto &observations_msg : other.observations_synced) {
      auto camera_info = CameraInfo::from(observations_msg.camera_info);
      camera_info_map.m_mutable().emplace(camera_info.imager_frame_id(), camera_info);
    }
    return camera_info_map;
  }

  // ==============================================================================
// from fvlam/marker.hpp
// ==============================================================================

  template<>
  MapEnvironment MapEnvironment::from<fiducial_vlam_msgs::msg::MapEnvironment>(
    fiducial_vlam_msgs::msg::MapEnvironment &other)
  {
    return MapEnvironment{other.description,
                          other.marker_dictionary_id,
                          other.marker_length};
  }

  template<>
  fiducial_vlam_msgs::msg::MapEnvironment MapEnvironment::to<fiducial_vlam_msgs::msg::MapEnvironment>() const
  {
    return fiducial_vlam_msgs::msg::MapEnvironment{}
      .set__description(description_)
      .set__marker_dictionary_id(marker_dictionary_id_)
      .set__marker_length(marker_length_);
  }

  template<>
  Marker Marker::from<fiducial_vlam_msgs::msg::Marker>(
    fiducial_vlam_msgs::msg::Marker &other)
  {
    return Marker{other.id,
                  Transform3WithCovariance::from(other.t_map_marker),
                  other.is_fixed};
  }

  template<>
  fiducial_vlam_msgs::msg::Marker Marker::to<fiducial_vlam_msgs::msg::Marker>() const
  {
    return fiducial_vlam_msgs::msg::Marker{}
      .set__id(id_)
      .set__is_fixed(is_fixed_)
      .set__t_map_marker(t_map_marker_.to<geometry_msgs::msg::PoseWithCovariance>());
  }

  template<>
  visualization_msgs::msg::Marker Marker::to<visualization_msgs::msg::Marker>() const
  {
    return visualization_msgs::msg::Marker{}
      .set__id(id_)
      .set__pose(t_map_marker().tf().to<geometry_msgs::msg::Pose>())
      .set__type(visualization_msgs::msg::Marker::CUBE)
      .set__action(visualization_msgs::msg::Marker::ADD)
      .set__scale(geometry_msgs::msg::Vector3{}.set__x(0.1).set__y(0.1).set__z(0.01))
      .set__color(std_msgs::msg::ColorRGBA{}.set__r(1.f).set__g(1.f).set__b(0.f).set__a(1.f));
  }

  template<>
  MarkerMap MarkerMap::from<fiducial_vlam_msgs::msg::Map>(
    fiducial_vlam_msgs::msg::Map &other)
  {
    MarkerMap map{MapEnvironment::from(other.map_environment)};
    for (auto &marker : other.markers) {
      map.add_marker(Marker::from(marker));
    }
    return map;
  }

  template<>
  fiducial_vlam_msgs::msg::Map MarkerMap::to<fiducial_vlam_msgs::msg::Map>() const
  {
    auto msg = fiducial_vlam_msgs::msg::Map{}
      .set__map_environment(map_environment().to<fiducial_vlam_msgs::msg::MapEnvironment>());

    for (auto &id_marker_pair : m_) {
      msg.markers.emplace_back(id_marker_pair.second.to<fiducial_vlam_msgs::msg::Marker>());
    }

    return msg;
  }

// ==============================================================================
// from fvlam/observation.hpp
// ==============================================================================

  template<>
  Stamp Stamp::from<builtin_interfaces::msg::Time>(
    builtin_interfaces::msg::Time &other)
  {
    return Stamp{other.sec, other.nanosec};
  }

  template<>
  builtin_interfaces::msg::Time Stamp::to
    <builtin_interfaces::msg::Time>() const
  {
    return builtin_interfaces::msg::Time{}
      .set__sec(sec_)
      .set__nanosec(nanosec_);
  }

  template<>
  Observation Observation::from<fiducial_vlam_msgs::msg::Observation>(
    fiducial_vlam_msgs::msg::Observation &other)
  {
    return Observation{other.id,
                       other.x0, other.y0,
                       other.x1, other.y1,
                       other.x2, other.y2,
                       other.x3, other.y3};
  }

  template<>
  Observations Observations::from<fiducial_vlam_msgs::msg::Observations>(
    fiducial_vlam_msgs::msg::Observations &other)
  {
    fvlam::Observations observations{other.camera_info.imager_frame_id};
    for (auto &obs : other.observations) {
      auto observation = Observation::from(obs);
      observations.v_mutable().emplace_back(observation);
    }
    return observations;
  }

  template<>
  ObservationsSynced ObservationsSynced::from<fiducial_vlam_msgs::msg::ObservationsSynced>(
    fiducial_vlam_msgs::msg::ObservationsSynced &other)
  {
    auto stamp = Stamp::from(other.header.stamp);
    auto observations_synced = ObservationsSynced{stamp, other.header.frame_id};
    for (auto &observations_msg : other.observations_synced) {
      observations_synced.v_mutable().emplace_back(Observations::from(observations_msg));
    }
    return observations_synced;
  }

  template<>
  std::vector<fiducial_vlam_msgs::msg::Observation> Observations::to
    <std::vector<fiducial_vlam_msgs::msg::Observation>>() const
  {
    std::vector<fiducial_vlam_msgs::msg::Observation> msgs;
    for (auto observation : v_) {
      fiducial_vlam_msgs::msg::Observation msg;
      msg.id = observation.id();
      msg.x0 = observation.corners_f_image()[0].x();
      msg.x1 = observation.corners_f_image()[1].x();;
      msg.x2 = observation.corners_f_image()[2].x();;
      msg.x3 = observation.corners_f_image()[3].x();;
      msg.y0 = observation.corners_f_image()[0].y();;
      msg.y1 = observation.corners_f_image()[1].y();;
      msg.y2 = observation.corners_f_image()[2].y();;
      msg.y3 = observation.corners_f_image()[3].y();;
      msgs.emplace_back(msg);
    }
    return msgs;
  }
}
