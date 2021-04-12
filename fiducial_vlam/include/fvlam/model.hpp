#pragma once

#include "fvlam/camera_info.hpp"
#include "fvlam/logger.hpp"
#include "fvlam/marker.hpp"
#include "fvlam/observation.hpp"


namespace fvlam
{

  struct MapEnvironmentGen
  {
    static MapEnvironment Default(); //
  };

  struct CameraInfoMapGen
  {
    static CameraInfoMap Simulation(); //
    static CameraInfoMap Dual(); //
    static CameraInfoMap DualWideAngle(); //
  };

  struct CamerasGen
  {
    static std::vector<Transform3> SpinAboutZAtOriginFacingOut(int n); //
    static std::vector<Transform3> LookingDownZ(double z); //
    static std::vector<Transform3> CircleInXYPlaneFacingAlongZ(int n, double radius, double z_offset,
                                                               bool facing_z_plus_not_z_negative); //
  };

  template<class Target>
  struct TargetsGen
  {
    static std::vector<Target> TargetsFromTransform3s(std::vector<Transform3> transform3s); //
    static std::vector<Target> CircleInXYPlaneFacingOrigin(int n, double radius); //
    static std::vector<Target> OriginLookingUp(); //
    static std::vector<Target> CircleInXYPlaneFacingAlongZ(int n, double radius, double z_offset,
                                                           bool facing_z_plus_not_z_negative); //
  };

  using MarkersGen = TargetsGen<Marker>;

// ==============================================================================
// MarkerObservations class
// ==============================================================================

  class MarkerObservations
  {
    uint64_t camera_index_;
    Transform3 t_map_camera_;
    ObservationsSynced observations_synced_;

    static ObservationsSynced gen_observations_synced(const MapEnvironment &map_environment,
                                                      const CameraInfoMap &camera_info_map,
                                                      const Transform3 &t_map_camera,
                                                      const std::vector<Marker> &markers);

  public:
    MarkerObservations(const MapEnvironment &map_environment,
                       const CameraInfoMap &camera_info_map,
                       Transform3 t_map_camera,
                       const std::vector<Marker> &markers,
                       uint64_t camera_index) :
      camera_index_{camera_index},
      t_map_camera_{std::move(t_map_camera)},
      observations_synced_(gen_observations_synced(map_environment, camera_info_map, t_map_camera, markers))
    {}

    MarkerObservations(uint64_t camera_index,
                       Transform3 t_map_camera,
                       ObservationsSynced observations_synced) :
      camera_index_{camera_index},
      t_map_camera_{std::move(t_map_camera)},
      observations_synced_{std::move(observations_synced)}
    {}

    auto &camera_index() const
    { return camera_index_; }

    auto &t_map_camera() const
    { return t_map_camera_; }

    auto &observations_synced() const
    { return observations_synced_; }
  };

// ==============================================================================
// Model class
// ==============================================================================

  template<class Environment, class Target, class TargetObservations>
  class Model
  {
    Environment environment_;
    CameraInfoMap camera_info_map_;
    std::vector<Target> targets_;
    std::vector<TargetObservations> target_observations_list_;

    static std::vector<TargetObservations> gen_target_observations_list(const Environment &environment,
                                                                        const CameraInfoMap &camera_info_map,
                                                                        const std::vector<Transform3> &t_map_cameras,
                                                                        const std::vector<Target> &targets)
    {
      std::vector<TargetObservations> target_observations_list;
      for (size_t i = 0; i < t_map_cameras.size(); i += 1) {
        target_observations_list.template emplace_back(
          TargetObservations(environment, camera_info_map, t_map_cameras[i], targets, i));
      }
      return target_observations_list;
    }

  public:
    Model(Environment environment,
          CameraInfoMap camera_info_map,
          const std::vector<Transform3> &t_map_cameras,
          std::vector<Target> targets) :
      environment_{std::move(environment)},
      camera_info_map_{std::move(camera_info_map)},
      targets_{std::move(targets)},
      target_observations_list_{gen_target_observations_list(environment_, camera_info_map_, t_map_cameras, targets_)}
    {}

    using Maker = std::function<Model<Environment, Target, TargetObservations>(void)>;

    auto &environment() const
    { return environment_; }

    auto &camera_info_map() const
    { return camera_info_map_; }

    auto &targets() const
    { return targets_; }

    auto &target_observations_list() const
    { return target_observations_list_; }
  };

  using MarkerModel = Model<MapEnvironment, Marker, MarkerObservations>;

  struct MarkerModelGen
  {
    static MarkerModel::Maker MonoSpinCameraAtOrigin(); //
    static MarkerModel::Maker DualSpinCameraAtOrigin(); //
    static MarkerModel::Maker MonoParallelGrid(); //
    static MarkerModel::Maker DualParallelGrid(); //
    static MarkerModel::Maker MonoParallelCircles(); //
    static MarkerModel::Maker DualParallelCircles(); //
    static MarkerModel::Maker DualWideSingleCamera(); //
  };

  struct ModelKey
  {
    static std::uint64_t camera(std::size_t idx); //
    static std::uint64_t marker(std::size_t idx); //
    static std::uint64_t corner(std::uint64_t marker_key, int corner_idx); //
    static std::uint64_t marker_from_corner(std::uint64_t corner_key); //
  };

// ==============================================================================
// Runner class
// ==============================================================================

  class MarkerModelRunnerSimple
  {
  public:
    struct Config
    {
      double equals_tolerance_ = 1.0e-2;
      fvlam::Logger::Levels logger_level_ = fvlam::Logger::Levels::level_warn;
    };

    template<class Uut>
    using UutMaker = std::function<Uut(MarkerModelRunnerSimple &)>;

  private:
    Config cfg_;
    LoggerCout logger_;
    MarkerModel model_;

  public:
    MarkerModelRunnerSimple() = delete; //
    MarkerModelRunnerSimple(const MarkerModelRunnerSimple &) = delete; //
    MarkerModelRunnerSimple(MarkerModelRunnerSimple &&) = delete;

    MarkerModelRunnerSimple(Config cfg,
                            MarkerModel::Maker model_maker);

    template<class TestMaker>
    bool run(TestMaker test_maker)
    {
      auto test = test_maker(*this);
      return test();
    }

    template<class TestMaker, class UutMaker>
    bool run(TestMaker test_maker, UutMaker uut_maker)
    {
      auto test = test_maker(*this);
      auto uut = uut_maker(*this);
      return test(std::move(uut));
    }

    auto &cfg() const
    { return cfg_; }

    Logger &logger()
    { return logger_; }

    auto &model() const
    { return model_; }
  };

  class MarkerModelRunner
  {
  public:
    struct Config
    {
      double r_sampler_sigma_ = 0.001;
      double t_sampler_sigma_ = 0.001;
      double u_sampler_sigma_ = 0.001;

      double equals_tolerance_ = 1.0e-2;

      fvlam::Logger::Levels logger_level_ = fvlam::Logger::Levels::level_warn;
    };

    template<class Uut>
    using UutMaker = std::function<Uut(MarkerModelRunner &)>;

  private:
    Config cfg_;
    LoggerCout logger_;
    MarkerModel model_;
    fvlam::MarkerMap map_;
    std::vector<Marker> markers_perturbed_;
    std::vector<fvlam::MarkerObservations> marker_observations_list_perturbed_;

    static fvlam::MarkerMap gen_map(
      const fvlam::MarkerModel &model);

    static std::vector<Marker> gen_markers_perturbed(
      const fvlam::MarkerModel &model,
      double r_sampler_sigma,
      double t_sampler_sigma);

    static std::vector<fvlam::MarkerObservations> gen_marker_observations_list_perturbed(
      const fvlam::MarkerModel &model,
      double r_sampler_sigma,
      double t_sampler_sigma,
      double point2_sampler_sigma);

  public:
    MarkerModelRunner() = delete; //
    MarkerModelRunner(const MarkerModelRunner &) = delete; //
    MarkerModelRunner(MarkerModelRunner &&) = delete;

    MarkerModelRunner(Config cfg,
                      MarkerModel::Maker model_maker);

    template<class TestMaker>
    bool run(TestMaker test_maker)
    {
      auto test = test_maker(*this);
      return test();
    }

    template<class TestMaker, class UutMaker>
    bool run(TestMaker test_maker, UutMaker uut_maker)
    {
      auto test = test_maker(*this);
      auto uut = uut_maker(*this);
      return test(std::move(uut));
    }

    auto &cfg() const
    { return cfg_; }

    Logger &logger()
    { return logger_; }

    auto &model() const
    { return model_; }

    auto &map() const
    { return map_; }

    auto &marker_observations_list_perturbed() const
    { return marker_observations_list_perturbed_; }
  };
}
