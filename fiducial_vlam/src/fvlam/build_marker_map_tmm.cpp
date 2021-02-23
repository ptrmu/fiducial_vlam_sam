#pragma ide diagnostic ignored "modernize-use-nodiscard"

#include <memory>

#include "fvlam/build_marker_map_interface.hpp"
#include "fvlam/camera_info.hpp"
#include "fvlam/factors_gtsam.hpp"
#include "fvlam/logger.hpp"
#include "fvlam/marker.hpp"
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/sfm/ShonanAveraging.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/InitializePose3.h>
#include <opencv2/calib3d/calib3d.hpp>

namespace fvlam
{
  template<class SolveTmm>
  class MarkerMarkerGraph
  {
  public:
    class IdIxList
    {
      std::vector<std::uint64_t> to_id_{};
      std::map<std::uint64_t, std::size_t> to_ix_{};

    public:
      constexpr static std::uint64_t bad_id = UINT64_MAX;
      constexpr static std::uint64_t bad_ix = SIZE_MAX;

      std::uint64_t to_id(std::size_t ix) const
      {
        return ix >= to_id_.size() ? bad_id : to_id_[ix];
      }

      std::size_t to_ix(std::uint64_t id) const
      {
        auto it = to_ix_.find(id);
        return it == to_ix_.end() ? bad_ix : it->second;
      }

      std::size_t add(std::uint64_t id)
      {
        auto ix = to_id_.size();
        to_id_.emplace_back(id);
        to_ix_.emplace(id, ix);
        return ix;
      }

      std::size_t size() const
      {
        return to_id_.size();
      }
    };

  private:
    std::map<std::uint64_t, fvlam::Marker> fixed_markers_{};
    std::map<std::uint64_t, std::map<std::uint64_t, SolveTmm>> solve_tmm_map_{};
    std::map<std::uint64_t, std::set<std::uint64_t>> back_links_{};

  public:
    explicit MarkerMarkerGraph(const MarkerMap &map_initial)
    {
      for (auto &marker : map_initial) {
        if (marker.second.is_fixed()) {
          fixed_markers_.emplace(marker);
        }
      }
    }

    auto &fixed_markers() const
    { return fixed_markers_; }

    // This search order has the benefit of assigning indices in the same order as id values which
    // could be easier for debugging.
    void depth_first(std::uint64_t id, IdIxList &visited)
    {
      // If this id has been evaluated or is in the process of being evaluated, then ignore it.
      if (visited.to_ix(id) != IdIxList::bad_ix) {
        return;
      }

      // Mark this id as having been visited and that it is linked to a fixed marker
      visited.add(id);

      // Follow the forward links
      auto f_links = solve_tmm_map_.find(id);
      if (f_links != solve_tmm_map_.end()) {
        for (auto &f_link : f_links->second) {
          depth_first(f_link.first, visited);
        }
      }

      // follow the backward links
      auto b_links = back_links_.find(id);
      if (b_links != back_links_.end()) {
        for (auto &b_link : b_links->second) {
          depth_first(b_link, visited);
        }
      }
    }

    IdIxList find_linked_nodes()
    {
      IdIxList visited{};

#if 1 // breadth first search
      std::vector<std::uint64_t> work_list{};

      // Add any fixed markers to the work list
      for (auto &id_marker_pair : fixed_markers_) {
        work_list.emplace_back(id_marker_pair.first);
      }

      // Follow links from each entry in the work list.
      for (std::size_t i = 0; i < work_list.size(); i += 1) {
        auto id = work_list[i];

        // Skip following links if we have already visited this node
        if (visited.to_ix(id) != IdIxList::bad_ix) {
          continue;
        }

        // Mark this id as having been visited and that it is linked to a fixed marker
        visited.add(id);

        // Follow the forward links
        auto f_links = solve_tmm_map_.find(id);
        if (f_links != solve_tmm_map_.end()) {
          for (auto &f_link : f_links->second) {
            if (visited.to_ix(f_link.first) == IdIxList::bad_ix) {
              work_list.emplace_back(f_link.first);
            }
          }
        }

        // follow the backward links
        auto b_links = back_links_.find(id);
        if (b_links != back_links_.end()) {
          for (auto &b_link : b_links->second) {
            if (visited.to_ix(b_link) == IdIxList::bad_ix) {
              work_list.emplace_back(b_link);
            }
          }
        }
      }
#else // depth first
      // Start with any fixed markers:
      for (auto &id_marker_pair : fixed_markers_) {
        depth_first(id_marker_pair.first, visited);
      }

#endif
      return visited;
    }

    SolveTmm *lookup(std::uint64_t id0, std::uint64_t id1)
    {
      if (id0 > id1) {
        std::swap(id0, id1);
      }

      auto f_links = solve_tmm_map_.find(id0);
      if (f_links == solve_tmm_map_.end()) {
        return nullptr;
      }

      auto f_link = f_links->second.find(id1);
      if (f_link == f_links->second.end()) {
        return nullptr;
      }

      return &f_link->second;
    }

    SolveTmm &add_or_lookup(std::uint64_t id0, std::uint64_t id1, std::function<SolveTmm(void)> solve_tmm_factory)
    {
      if (id0 > id1) {
        std::swap(id0, id1);
      }

      // Look for thee forward link
      auto f_links = solve_tmm_map_.find(id0);
      if (f_links == solve_tmm_map_.end()) {
        solve_tmm_map_.emplace(id0, std::map<std::uint64_t, SolveTmm>{});
        f_links = solve_tmm_map_.find(id0);
      }

      auto f_link = f_links->second.find(id1);
      if (f_link != f_links->second.end()) {
        return f_link->second;
      }

      // Insert a forward link
      f_links->second.emplace(id1, solve_tmm_factory());
      f_link = f_links->second.find(id1);

      // Insert a back link
      auto b_links = back_links_.find(id1);
      if (b_links == back_links_.end()) {
        back_links_.emplace(id1, std::set<std::uint64_t>{});
        b_links = back_links_.find(id1);
      }

      auto b_link = b_links->second.find(id0);
      if (b_link == b_links->second.end()) {
        b_links->second.emplace(id0);
      }

      return f_link->second;
    }
  };

  class BuildMarkerMapTmm : public BuildMarkerMapInterface
  {

    using SolveTmmGraph = MarkerMarkerGraph<std::unique_ptr<SolveTMarker0Marker1Interface>>;

    const BuildMarkerMapTmmContext tmm_context_;
    Logger &logger_;
    MarkerMap map_initial_;

    SolveTmmGraph solve_tmm_graph_;
    BuildMarkerMapTmmContext::BuildError error_;

    static fvlam::Marker find_fixed_marker(const MarkerMap &map)
    {
      for (auto &marker : map) {
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
          return gtsam::noiseModel::Gaussian::Covariance(
            GtsamUtil::cov_gtsam_from_ros(twc.tf().to<gtsam::Pose3>(), twc.cov()));

        case BuildMarkerMapTmmContext::NoiseStrategy::minimum:
          Transform3::MuVector t_sigmas = GtsamUtil::cov_gtsam_from_ros(twc.tf().to<gtsam::Pose3>(),
                                                                        twc.cov()).diagonal().array().sqrt();
          return gtsam::noiseModel::Diagonal::Sigmas((Transform3::MuVector()
            << minimum_sigma(t_sigmas.head<3>(), tmm_context_.mm_between_factor_noise_fixed_sigma_r_, isotropic),
            minimum_sigma(t_sigmas.tail<3>(), tmm_context_.mm_between_factor_noise_fixed_sigma_t_, isotropic))
                                                       .finished());
      }
    }

    gtsam::NonlinearFactorGraph load_pose_graph(const SolveTmmGraph::IdIxList &idix_list)
    {
      gtsam::NonlinearFactorGraph pose_graph{};

      for (std::size_t ix0 = 0; ix0 < idix_list.size(); ix0 += 1)
        for (std::size_t ix1 = ix0 + 1; ix1 < idix_list.size(); ix1 += 1) {
          auto solve_tmm = solve_tmm_graph_.lookup(idix_list.to_id(ix0), idix_list.to_id(ix1));
          if (solve_tmm) {
            auto t_marker0_marker1 = (*solve_tmm)->t_marker0_marker1();

            auto noise_model = determine_between_factor_noise_model(t_marker0_marker1,
                                                                    tmm_context_.try_shonan_initialization_);

            // The t_marker0_marker1 measurements are always recorded with the marker with the
            // lower id first - as the "world" marker and the higher id is the "body" marker. If
            // when we reassign ids, the higher id ends up as the "world" marker, then the
            // measurement has to be inverted.
            bool interchange = idix_list.to_id(ix0) > idix_list.to_id(ix1);
            auto ix0_bf = interchange ? ix1 : ix0;
            auto ix1_bf = interchange ? ix0 : ix1;

            pose_graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
              ix0_bf, ix1_bf, t_marker0_marker1.tf().to<gtsam::Pose3>(),
              noise_model);
          }
        }

      // Add the prior for the fixed nodes.
      for (auto &id_marker_pair : solve_tmm_graph_.fixed_markers()) {
        pose_graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
          idix_list.to_ix(id_marker_pair.first),
          id_marker_pair.second.t_map_marker().tf().to<gtsam::Pose3>(),
          gtsam::noiseModel::Constrained::MixedSigmas(gtsam::Pose3::TangentVector::Zero()));
      }

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
    void get_shonan_rotations(const SolveTmmGraph::IdIxList &idix_list,
                              const gtsam::NonlinearFactorGraph &pose_graph,
                              gtsam::Values &initial_poses)
    {
      // Optimize rotations.
      auto shonan_measurements = load_shonan_measurements(pose_graph);
      gtsam::ShonanAveraging3 shonan(shonan_measurements);
      auto shonan_initial = shonan.initializeRandomly();
      auto shonan_result = shonan.run(shonan_initial);
      error_.shonan_error_ = shonan_result.second;

      // Find the rotation that the shonan algorithm returned for the fixed
      // marker. Then figure out the delta rotation to rotate that shonan
      // rotation to the fixed rotation. Then apply this rotation to all
      // shonan rotations as we are entering them in the initial values.
      auto r_world_shonan{gtsam::Rot3::identity()};
      auto fixed_marker_ix = idix_list.to_ix(solve_tmm_graph_.fixed_markers().begin()->first);
      for (const auto &key_value : shonan_result.first) {
        if (key_value.key == fixed_marker_ix) {
          r_world_shonan =
            solve_tmm_graph_.fixed_markers().begin()->second.t_map_marker().tf().r().to<gtsam::Rot3>() *
            shonan_result.first.at<typename gtsam::Pose3::Rotation>(key_value.key).inverse();
          break;
        }
      }

      // Upgrade rotations in initial_poses
      for (const auto &key_value : shonan_result.first) {
        gtsam::Key key = key_value.key;
        const auto &rot = shonan_result.first.at<gtsam::Rot3>(key);
        auto rot_f_world = r_world_shonan * rot;
        logger_.debug() << key << " " << fvlam::Rotate3::from(rot_f_world).to_string();
        auto initializedPose = gtsam::Pose3{rot_f_world, initial_poses.at<gtsam::Pose3>(key).translation()};
        initial_poses.update(key, initializedPose);
      }
    }

    gtsam::Values load_pose_initial(const SolveTmmGraph::IdIxList &idix_list,
                                    const gtsam::NonlinearFactorGraph &pose_graph)
    {
      // initialize poses by the chordal method
      auto initial_poses = gtsam::InitializePose3::initialize(pose_graph);

      if (tmm_context_.try_shonan_initialization_) {
        get_shonan_rotations(idix_list, pose_graph, initial_poses);
      }
      return initial_poses;
    }

    std::unique_ptr<MarkerMap> load_map(const SolveTmmGraph::IdIxList &idix_list,
                                        const gtsam::NonlinearFactorGraph &pose_graph,
                                        const gtsam::Values &pose_result)
    {
//      auto map = std::make_unique<MarkerMap>(map_initial_.marker_length()); todo: Fix this
      auto map = std::make_unique<MarkerMap>();
      gtsam::Marginals marginals{GtsamUtil::construct_marginals(pose_graph, pose_result)};

      for (const auto &key_value : pose_result) {
        gtsam::Key key = key_value.key;
        auto id = idix_list.to_id(key);

        // Check to see if this is a fixed marker
        auto fixed = solve_tmm_graph_.fixed_markers().find(id);
        if (fixed != solve_tmm_graph_.fixed_markers().end()) {
          map->add_marker(fixed->second);
          continue;
        }

        auto t_map_marker = GtsamUtil::extract_transform3_with_covariance(marginals, pose_result, key);
        map->add_marker(Marker{id, t_map_marker});
      }

      return map;
    }

    void calc_remeasure_error(const MarkerMap &map)
    {
      double r_sum{0.0};
      double t_sum{0.0};
      uint64_t n{0};

      for (auto it0 = map.begin(); it0 != map.end(); ++it0)
        for (auto it1 = map.upper_bound(it0->first); it1 != map.end(); ++it1) {
          auto solve_tmm = solve_tmm_graph_.lookup(it0->first, it1->first);
          if (solve_tmm != nullptr) {
            auto tmm_meas = (*solve_tmm)->t_marker0_marker1().tf();
            auto tmm_calc = it0->second.t_map_marker().tf().inverse() *
                            it1->second.t_map_marker().tf();
            r_sum += tmm_calc.r().q().angularDistance(tmm_meas.r().q());
            t_sum += (tmm_calc.t().t() - tmm_meas.t().t()).norm();
            n += 1;
          }
        }

      if (n > 1) {
        r_sum /= n;
        t_sum /= n;
      }

      error_.r_remeasure_error_ = r_sum;
      error_.t_remeasure_error_ = t_sum;
    }

  public:
    BuildMarkerMapTmm() = delete;

    BuildMarkerMapTmm(BuildMarkerMapTmmContext tmm_context,
                      Logger &logger,
                      const MarkerMap &map_initial) :
      tmm_context_{std::move(tmm_context)},
      logger_{logger},
      map_initial_{map_initial},
      solve_tmm_graph_{map_initial},
      error_{}
    {}

    void process(const ObservationsSynced &observations_synced,
                 const CameraInfoMap &camera_info_map) override
    {
      // Walk through all the imagers
      for (auto &observations : observations_synced) {
        // Find the camera_info for this imager
        auto camera_info_it = camera_info_map.find(observations.imager_frame_id());
        if (camera_info_it != camera_info_map.end()) {
          // Walk through all pairs of observations
          for (std::size_t m0 = 0; m0 < observations.size(); m0 += 1)
            for (std::size_t m1 = m0 + 1; m1 < observations.size(); m1 += 1) {
              std::size_t m0r{m0};
              std::size_t m1r{m1};

              // Alawys do the transform calculation with the lower id first.
              if (observations[m1r].id() < observations[m0r].id()) {
                m0r = m1;
                m1r = m0;
              }

              // Find the appropriate SolveTmmInterface
              auto &solve_tmm = solve_tmm_graph_.add_or_lookup(observations[m0r].id(), observations[m1r].id(),
                                                               tmm_context_.solve_tmm_factory_);
              solve_tmm->accumulate(observations[m0r], observations[m1r], camera_info_it->second);
            }
        }
      }
    }

    std::unique_ptr<MarkerMap> build() override
    {
      auto idix_list = solve_tmm_graph_.find_linked_nodes();

      // Make sure there are some markers linked to the fixed markers.
      if (idix_list.size() == map_initial_.size()) {
        return std::make_unique<MarkerMap>(map_initial_);
      }

      // Prepare for full Pose optimization
      auto pose_graph = load_pose_graph(idix_list);
      if (logger_.output_debug()) {
//        pose_graph.print("pose_graph\n");
      }

      auto pose_initial = load_pose_initial(idix_list, pose_graph);
      if (logger_.output_debug()) {
//        pose_initial.print("pose_initial");
      }
      // Do the pose optimization
      gtsam::GaussNewtonParams params;
      if (logger_.output_debug()) {
        params.setVerbosity("TERMINATION");
      }

      gtsam::GaussNewtonOptimizer optimizer(pose_graph, pose_initial, params);
      auto pose_result = optimizer.optimize();
      error_.nonlinear_optimization_error_ = pose_graph.error(pose_result);

      auto built_map = load_map(idix_list, pose_graph, pose_result);
      calc_remeasure_error(*built_map);
      error_.valid_ = true; // Mark this error structure as having valid data.
      return built_map;
    }

    auto error() const
    { return error_; }
  };

  template<>
  std::unique_ptr<BuildMarkerMapInterface> make_build_marker_map<BuildMarkerMapTmmContext>(
    const BuildMarkerMapTmmContext &bmm_context,
    Logger &logger,
    const MarkerMap &map_initial)
  {
    return std::make_unique<BuildMarkerMapTmm>(bmm_context, logger, map_initial);
  }

  BuildMarkerMapTmmContext::BuildError BuildMarkerMapTmmContext::BuildError::from(
    const BuildMarkerMapInterface &bmm_interface,
    const MarkerMap &built_map)
  {
    (void) built_map;
    auto bmm_tmm = dynamic_cast<const BuildMarkerMapTmm *>(&bmm_interface);
    return bmm_tmm != nullptr ? bmm_tmm->error() : BuildMarkerMapTmmContext::BuildError{};
  }

  std::string BuildMarkerMapTmmContext::BuildError::to_string() const
  {
    return std::string{"shonan error:"} + std::to_string(shonan_error_) +
           " non-linear error:" + std::to_string(nonlinear_optimization_error_) +
           " remeasure error r:" + std::to_string(r_remeasure_error_) +
           " t:" + std::to_string(t_remeasure_error_);
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
      auto t_camera_marker0 = observation0.solve_t_camera_marker(camera_info, marker_length_);
      auto t_camera_marker1 = observation1.solve_t_camera_marker(camera_info, marker_length_);
      auto t_marker0_marker1 = t_camera_marker0.inverse() * t_camera_marker1;
      if (solve_tmm_context_.average_on_space_not_manifold_) {
        emac_algebra_.accumulate(t_marker0_marker1);
      } else {
        emac_group_.accumulate(t_marker0_marker1.mu());
      }
    }

    // Given the observations that have been added so far, create and return a marker_map.
    Transform3WithCovariance t_marker0_marker1() override
    {
      return Transform3WithCovariance{
        solve_tmm_context_.average_on_space_not_manifold_ ? emac_algebra_.mean() : Transform3(emac_group_.mean()),
        solve_tmm_context_.average_on_space_not_manifold_ ? emac_algebra_.cov() : emac_group_.cov()};
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
