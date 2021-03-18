

#include "fvlam/model.hpp"
#include <gtsam/geometry/Cal3DS2.h>
#include "opencv2/core.hpp"

namespace fvlam
{
  MapEnvironment MapEnvironmentGen::Default()
  {
    return fvlam::MapEnvironment{"TestMap", 0, 0.2};
  }

  CameraInfoMap CameraInfoMapGen::Simulation()
  {
    auto camera_info_base = CameraInfo{475, 475, 0, 400, 300};
    auto camera_info0 = CameraInfo("imager", camera_info_base, Transform3{});
    auto camera_info_map = CameraInfoMap{};
    camera_info_map.m_mutable().emplace(camera_info0.imager_frame_id(), camera_info0);
    return camera_info_map;
  }

  static CameraInfoMap centered_dual_camera(CameraInfo camera_info_base, double imager_offset)
  {
    auto camera_info0 = CameraInfo("left", camera_info_base,
                                   Transform3{Rotate3{}, Translate3{-imager_offset, 0, 0}});
    auto camera_info1 = CameraInfo("right", camera_info_base,
                                   Transform3{Rotate3{}, Translate3{imager_offset, 0, 0}});

    auto camera_info_map = CameraInfoMap{};
    camera_info_map.m_mutable().emplace(camera_info0.imager_frame_id(), camera_info0);
    camera_info_map.m_mutable().emplace(camera_info1.imager_frame_id(), camera_info1);

    return camera_info_map;
  }

  CameraInfoMap CameraInfoMapGen::Dual()
  {
    auto camera_info_base = CameraInfo{475, 475, 0, 400, 300};
    return centered_dual_camera(camera_info_base, 0.2);
  }

  CameraInfoMap CameraInfoMapGen::DualWideAngle()
  {
    auto camera_info_base = CameraInfo{100, 100, 0, 400, 300};
    return centered_dual_camera(camera_info_base, 0.2);
  }

  static std::vector<fvlam::Transform3> rotate_around_z(int n, const fvlam::Transform3 &base)
  {
    std::vector<fvlam::Transform3> pose_f_worlds{};
    double delta_rotz = M_PI * 2 / n;
    for (int i = 0; i < n; i += 1) {
      auto rotz = delta_rotz * i;
      pose_f_worlds.emplace_back(
        fvlam::Transform3(fvlam::Rotate3::RzRyRx(0., 0., rotz), fvlam::Translate3{0, 0, 0}) * base);
    }
    return pose_f_worlds;
  }

  static std::vector<fvlam::Transform3> circle_in_xy_plane_facing_along_z(int n, double radius, double z_offset,
                                                                          bool facing_z_plus_not_z_negative)
  {
    auto base_rotate = facing_z_plus_not_z_negative ? Rotate3{} : Rotate3::RzRyRx(M_PI, 0, 0);
    auto poses_in_circle = rotate_around_z(n, Transform3{base_rotate, Translate3{radius, 0., z_offset}});
    return poses_in_circle;
  }

  static std::vector<Marker> markers_from_transform3s(std::vector<Transform3> transform3s,
                                                      std::uint64_t id_base)
  {
    std::vector<Marker> markers;
    for (auto &transform3 : transform3s) {
      markers.emplace_back(Marker{id_base++, Transform3WithCovariance{transform3}});
    }
    return markers;
  }

  std::vector<Transform3> CamerasGen::SpinAboutZAtOriginFacingOut(int n)
  {
    Transform3 base{fvlam::Rotate3::RzRyRx(-M_PI_2, 0., -M_PI_2), fvlam::Translate3{0, 0, 0}};
    return rotate_around_z(n, base);
  }

  std::vector<Transform3> CamerasGen::LookingDownZ(double z)
  {
    auto p = Transform3{fvlam::Rotate3::RzRyRx(M_PI, 0., 0.), fvlam::Translate3{0, 0, z}};
    return std::vector<Transform3>{p};
  }

  std::vector<Transform3> CamerasGen::CircleInXYPlaneFacingAlongZ(int n, double radius, double z_offset,
                                                                  bool facing_z_plus_not_z_negative)
  {
    return circle_in_xy_plane_facing_along_z(n, radius, z_offset, facing_z_plus_not_z_negative);
  }

  template<>
  std::vector<Marker> MarkersGen::TargetsFromTransform3s(std::vector<Transform3> transform3s)
  {
    return markers_from_transform3s(transform3s, 0);
  }

  template<>
  std::vector<Marker> MarkersGen::CircleInXYPlaneFacingOrigin(int n, double radius)
  {
    return markers_from_transform3s(rotate_around_z(
      n, Transform3{Rotate3::RzRyRx(M_PI_2, 0., -M_PI_2), Translate3{radius, 0., 0.}}), 0);
  }

  template<>
  std::vector<Marker> MarkersGen::OriginLookingUp()
  {
    return markers_from_transform3s(std::vector<Transform3>{Transform3{}}, 0);
  }

  template<>
  std::vector<Marker> MarkersGen::CircleInXYPlaneFacingAlongZ(int n, double radius, double z_offset,
                                                              bool facing_z_plus_not_z_negative)
  {
    return markers_from_transform3s(circle_in_xy_plane_facing_along_z(
      n, radius, z_offset, facing_z_plus_not_z_negative), 0);
  }

// ==============================================================================
// MarkerObservations class
// ==============================================================================

  ObservationsSynced MarkerObservations::gen_observations_synced(const MapEnvironment &map_environment,
                                                                 const CameraInfoMap &camera_info_map,
                                                                 const Transform3 &t_map_camera,
                                                                 const std::vector<Marker> &markers)
  {
    auto observations_synced = ObservationsSynced{Stamp{}, "camera_frame"};

    for (auto &camera_info_pair : camera_info_map.m()) {
      const CameraInfo &camera_info = camera_info_pair.second;
      auto gtsam_camera_calibration = camera_info.to<gtsam::Cal3DS2>();

      auto project_t_world_marker_function = fvlam::Marker::project_t_world_marker(
        gtsam_camera_calibration,
        t_map_camera * camera_info.t_camera_imager(),
        map_environment.marker_length());

      auto observations = Observations{camera_info.imager_frame_id()};
      for (auto &marker : markers) {
        auto observation = project_t_world_marker_function(marker);
        if (observation.is_valid()) {
          observations.v_mutable().emplace_back(observation);
        }
      }
      observations_synced.v_mutable().emplace_back(observations);
    }

    return observations_synced;
  }

}