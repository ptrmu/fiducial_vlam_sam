#pragma ide diagnostic ignored "modernize-use-nodiscard"

#include <memory>
#include <fvlam/camera_info.hpp>

#include "fvlam/build_marker_map_interface.hpp"
#include "fvlam/marker.hpp"
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/sfm/ShonanAveraging.h>
#include <gtsam/slam/InitializePose3.h>
#include "opencv2/calib3d/calib3d.hpp"

namespace fvlam
{
  class BuildMarkerMapShonan : public BuildMarkerMapInterface
  {
    constexpr static std::uint64_t JointID_id0(std::uint64_t id)
    { return id / 1000000L; }; //
    constexpr static std::uint64_t JointID_id1(std::uint64_t id)
    { return id % 1000000L; }; //
    constexpr static std::uint64_t JointID(std::uint64_t id0, std::uint64_t id1)
    { return id0 * 1000000L + id1; }; //

    const BuildMarkerMapTmmContext tmm_context_;
    Logger &logger_;
    const double marker_length_;
    const fvlam::Marker fixed_marker_;

    std::map<std::uint64_t, std::unique_ptr<SolveTMarker0Marker1Interface>> solve_tmm_map_;
    BuildMarkerMapTmmContext::Error error_;

    static fvlam::Marker find_fixed_marker(const MarkerMap &map)
    {
      for (auto &marker : map.markers()) {
        if (marker.second.is_fixed()) {
          return marker.second;
        }
      }
      return fvlam::Marker{0, Transform3WithCovariance{}};
    }

    static Eigen::Vector3d minimum_sigma(Eigen::Vector3d sigmas,
                                  double minimum,
                                  bool isotropic)
    {
      if (isotropic) {
        auto sigma_max = sigmas.maxCoeff();
        if (minimum < sigma_max) {
          minimum = sigma_max;
        }
      }
      if (sigmas(0) < minimum) {
        sigmas(0) = minimum;
      }
      if (sigmas(1) < minimum) {
        sigmas(1) = minimum;
      }
      if (sigmas(2) < minimum) {
        sigmas(2) = minimum;
      }
      return sigmas;
    }

    gtsam::SharedNoiseModel determine_between_factor_noise_model(const fvlam::Transform3WithCovariance &twc,
                                                                 bool isotropic)
    {
      switch (tmm_context_.mm_between_factor_noise_strategy_) {
        default:
        case BuildMarkerMapTmmContext::NoiseStrategy::fixed:
          return gtsam::noiseModel::Diagonal::Sigmas((Transform3::MuVector()
            << Rotate3::MuVector::Constant(tmm_context_.mm_between_factor_noise_fixed_sigma_r_),
            Translate3::MuVector::Constant(tmm_context_.mm_between_factor_noise_fixed_sigma_t_)).finished());

        case BuildMarkerMapTmmContext::NoiseStrategy::estimation:
          return gtsam::noiseModel::Gaussian::Covariance(twc.cov());

        case BuildMarkerMapTmmContext::NoiseStrategy::minimum:
          Transform3::MuVector t_sigmas = twc.cov().diagonal().array().sqrt();
          return gtsam::noiseModel::Diagonal::Sigmas((Transform3::MuVector()
            << minimum_sigma(t_sigmas.head<3>(), tmm_context_.mm_between_factor_noise_fixed_sigma_r_, isotropic),
            minimum_sigma(t_sigmas.tail<3>(), tmm_context_.mm_between_factor_noise_fixed_sigma_t_, isotropic))
                                                       .finished());
      }
    }

    gtsam::NonlinearFactorGraph load_pose_graph()
    {
      gtsam::NonlinearFactorGraph pose_graph{};

      for (auto &solve_tmm : solve_tmm_map_) {
        std::uint64_t id0 = JointID_id0(solve_tmm.first);
        std::uint64_t id1 = JointID_id1(solve_tmm.first);

        auto t_marker0_marker1 = solve_tmm.second->t_marker0_marker1();
        auto noise_model = determine_between_factor_noise_model(t_marker0_marker1,
                                                                tmm_context_.try_shonan_initialization_);

        pose_graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
          id0, id1, t_marker0_marker1.tf().to<gtsam::Pose3>(),
          noise_model);
      }

      // Add the prior for the fixed node.
      pose_graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
        fixed_marker_.id(), fixed_marker_.t_world_marker().tf().to<gtsam::Pose3>(),
        gtsam::noiseModel::Constrained::MixedSigmas(gtsam::Pose3::TangentVector::Zero()));

      return pose_graph;
    }

    static gtsam::ShonanAveraging3::Measurements load_shonan_measurements(const gtsam::NonlinearFactorGraph &pose_graph)
    {
      gtsam::ShonanAveraging3::Measurements measurements{};

      for (auto &factor : pose_graph) {
        auto bf = dynamic_cast<const gtsam::BetweenFactor<gtsam::Pose3> *>(factor.get());
        if (bf != nullptr) {
          auto measurement = gtsam::BinaryMeasurement<gtsam::Rot3>{
            bf->key1(), bf->key2(),
            bf->measured().rotation(),
            gtsam::noiseModel::Diagonal::Sigmas(bf->noiseModel()->sigmas().head<3>())};
          measurements.emplace_back(measurement);
        }
      }

      return measurements;
    }

    // Given the relative marker measurements, find the shonan rotations that optimally solve the
    // system. Then merge these rotations into the initialization that has already been done
    // by the chordal method.
    void get_shonan_rotations(const gtsam::NonlinearFactorGraph &pose_graph,
                              gtsam::Values &initial_poses)
    {
      // Optimize rotations.
      auto shonan_measurements = load_shonan_measurements(pose_graph);
      gtsam::ShonanAveraging3 shonan(shonan_measurements);
      auto shonan_initial = shonan.initializeRandomly();
      auto shonan_result = shonan.run(shonan_initial);

      error_.shonan_error = shonan_result.second;
      logger_.info() << "error of Shonan Averaging " << shonan_result.second << std::endl;

      // Find the rotation that the shonan algorithm returned for the fixed
      // marker. Then figure out the delta rotation to rotate that shonan
      // rotation to the fixed rotation. Then apply this rotation to all
      // shonan rotations as we are entering them in the initial values.
      auto r_world_shonan{gtsam::Rot3::identity()};
      for (const auto &key_value : shonan_result.first) {
        if (key_value.key == fixed_marker_.id()) {
          r_world_shonan = fixed_marker_.t_world_marker().tf().r().to<gtsam::Rot3>() *
                           shonan_result.first.at<typename gtsam::Pose3::Rotation>(key_value.key).inverse();
          break;
        }
      }


      // Upgrade rotations in initial_poses
      for (const auto &key_value : shonan_result.first) {
        gtsam::Key key = key_value.key;
        const auto &rot = shonan_result.first.at<gtsam::Rot3>(key);
        auto rot_f_world = r_world_shonan * rot;
        logger_.debug() << key << " " << fvlam::Rotate3::from(rot_f_world).to_string() << std::endl;
        auto initializedPose = gtsam::Pose3{rot_f_world, initial_poses.at<gtsam::Pose3>(key).translation()};
        initial_poses.update(key, initializedPose);
      }
    }

    gtsam::Values load_pose_initial(const gtsam::NonlinearFactorGraph &pose_graph)
    {
      // initialize poses by the chordal method
      auto initial_poses = gtsam::InitializePose3::initialize(pose_graph);

      if (tmm_context_.try_shonan_initialization_) {
        get_shonan_rotations(pose_graph, initial_poses);
      }
      return initial_poses;
    }

    std::unique_ptr<MarkerMap> load_map(const gtsam::NonlinearFactorGraph &pose_graph,
                                        const gtsam::Values &pose_result)
    {
      auto map = std::make_unique<MarkerMap>(marker_length_);
      gtsam::Marginals marginals(pose_graph, pose_result);

      for (const auto &key_value : pose_result) {
        gtsam::Key key = key_value.key;
        if (key == fixed_marker_.id()) {
          map->add_marker(fixed_marker_);
        } else {
          const auto &pose = pose_result.at<gtsam::Pose3>(key);
          auto cov = marginals.marginalCovariance(key);

          map->add_marker(Marker{key,
                                 Transform3WithCovariance{Transform3::from(pose), cov}});
        }
      }

      return map;
    }

    void calc_remeasure_error(const MarkerMap &map)
    {
      double r_sum{0.0};
      double t_sum{0.0};
      uint64_t n{0};

      for (auto it0 = map.markers().begin(); it0 != map.markers().end(); ++it0)
        for (auto it1 = map.markers().upper_bound(it0->first); it1 != map.markers().end(); ++it1) {
          auto it_tmm = solve_tmm_map_.find(JointID(it0->first, it1->first));
          if (it_tmm != solve_tmm_map_.end()) {
            auto tmm_meas = it_tmm->second->t_marker0_marker1().tf();
            auto tmm_calc = it0->second.t_world_marker().tf().inverse() *
                            it1->second.t_world_marker().tf();
            r_sum += tmm_calc.r().q().angularDistance(tmm_meas.r().q());
            t_sum += (tmm_calc.t().t() - tmm_meas.t().t()).norm();
            n += 1;
          }
        }

      if (n > 1) {
        r_sum /= n;
        t_sum /= n;
      }

      error_.r_remeasure_error = r_sum;
      error_.t_remeasure_error = t_sum;
    }

  public:
    BuildMarkerMapShonan() = delete;

    BuildMarkerMapShonan(BuildMarkerMapTmmContext tmm_context,
                         Logger &logger,
                         const MarkerMap &map_initial) :
      tmm_context_{std::move(tmm_context)},
      logger_{logger},
      marker_length_{map_initial.marker_length()},
      fixed_marker_{find_fixed_marker(map_initial)},
      solve_tmm_map_{},
      error_{}
    {}

    void process(const Observations &observations,
                 const CameraInfo &camera_info) override
    {
      auto &obs = observations.observations();

      // Walk through all pairs of observations
      for (std::size_t m0 = 0; m0 < observations.size(); m0 += 1)
        for (std::size_t m1 = m0 + 1; m1 < observations.size(); m1 += 1) {
          std::size_t m0r{m0};
          std::size_t m1r{m1};

          // Alawys do the transform calculation with the lower id first.
          if (obs[m1r].id() < obs[m0r].id()) {
            m0r = m1;
            m1r = m0;
          }

          // Find the appropriate SolveTmmInterface
          std::uint64_t joint_id = JointID(obs[m0r].id(), obs[m1r].id());
          auto it = solve_tmm_map_.find(joint_id);
          if (it == solve_tmm_map_.end()) {
            auto res = solve_tmm_map_.emplace(joint_id, tmm_context_.solve_tmm_factory_());
            assert(res.second);
            it = res.first;
          }
          it->second->accumulate(obs[m0r], obs[m1r], camera_info);
        }
    }

    std::unique_ptr<MarkerMap> build() override
    {
      // Prepare for full Pose optimization
      auto pose_graph = load_pose_graph();
      if (logger_.output_debug()) {
        pose_graph.print("pose_graph\n");
      }

      auto pose_initial = load_pose_initial(pose_graph);
      if (logger_.output_debug()) {
        pose_initial.print("pose_initial");
      }
      // Do the pose optimization
      gtsam::GaussNewtonParams params;
      if (logger_.output_debug()) {
        params.setVerbosity("TERMINATION");
      }

      gtsam::GaussNewtonOptimizer optimizer(pose_graph, pose_initial, params);
      auto pose_result = optimizer.optimize();
      error_.nonlinear_optimization_error = pose_graph.error(pose_result);
      logger_.info() << "Non-linear optimization error = " << error_.nonlinear_optimization_error << std::endl;

      auto built_map = load_map(pose_graph, pose_result);
      calc_remeasure_error(*built_map);
      return built_map;
    }

    auto error() const
    { return error_; }
  };

  template<>
  std::unique_ptr<BuildMarkerMapInterface> make_build_marker_map<BuildMarkerMapTmmContext>(
    const BuildMarkerMapTmmContext &tmm_context,
    Logger &logger,
    const MarkerMap &map_initial)
  {
    return std::make_unique<BuildMarkerMapShonan>(tmm_context, logger, map_initial);
  }

  BuildMarkerMapTmmContext::Error BuildMarkerMapTmmContext::get_error(
    const BuildMarkerMapInterface &Bmm,
    const MarkerMap &built_map)
  {
    (void) built_map;
    auto bmm_tmm = dynamic_cast<const BuildMarkerMapShonan *>(&Bmm);
    return bmm_tmm != nullptr ? bmm_tmm->error() : BuildMarkerMapTmmContext::Error{};
  }

// ==============================================================================
// SolveTmmCvSolvePnp class
// ==============================================================================

  class SolveTmmCvSolvePnp : public SolveTMarker0Marker1Interface
  {
    const SolveTmmContextCvSolvePnp solve_tmm_context_;
    double marker_length_;
    EstimateTransform3MeanAndCovariance emac_algebra_; // Averaging in the vector space
    EstimateMeanAndCovariance<Transform3::MuVector> emac_group_; // Averaging on the manifold

  public:
    SolveTmmCvSolvePnp(const SolveTmmContextCvSolvePnp &solve_tmm_context,
                       double marker_length) :
      solve_tmm_context_{solve_tmm_context},
      marker_length_{marker_length},
      emac_algebra_{}, emac_group_{}
    {}

    void accumulate(const Observation &observation0,
                    const Observation &observation1,
                    const CameraInfo &camera_info) override
    {
      auto solve_t_camera_marker_function = fvlam::Marker::solve_t_camera_marker<CvCameraCalibration>
        (camera_info.to<CvCameraCalibration>(), marker_length_);
      auto t_camera_marker0 = solve_t_camera_marker_function(observation0).t_world_marker().tf();
      auto t_camera_marker1 = solve_t_camera_marker_function(observation1).t_world_marker().tf();
      auto t_marker0_marker1 = t_camera_marker0.inverse() * t_camera_marker1;
      if (solve_tmm_context_.average_on_space_not_manifold) {
        emac_algebra_.accumulate(t_marker0_marker1);
      } else {
        emac_group_.accumulate(t_marker0_marker1.mu());
      }
    }

    // Given the observations that have been added so far, create and return a marker_map.
    Transform3WithCovariance t_marker0_marker1() override
    {
      return Transform3WithCovariance{
        solve_tmm_context_.average_on_space_not_manifold ? emac_algebra_.mean() : Transform3(emac_group_.mean()),
        solve_tmm_context_.average_on_space_not_manifold ? emac_algebra_.cov() : emac_group_.cov()};
    }
  };

  template<>
  SolveTMarker0Marker1Factory make_solve_tmm_factory<SolveTmmContextCvSolvePnp>
    (const SolveTmmContextCvSolvePnp &solve_tmm_context,
     double marker_length)
  {

    return [
      solve_tmm_context{solve_tmm_context},
      marker_length
    ]() -> std::unique_ptr<SolveTMarker0Marker1Interface>
    {
      return std::make_unique<SolveTmmCvSolvePnp>(solve_tmm_context, marker_length);
    };
  }
}
