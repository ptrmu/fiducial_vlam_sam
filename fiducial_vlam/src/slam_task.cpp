
#include "fiducial_math.hpp"

//#define ENABLE_TIMING

#include "cv_utils.hpp"
#include <gtsam/base/timing.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include "gtsam/inference/Symbol.h"
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include "gtsam_math.hpp"
#include "map.hpp"
#include "observation.hpp"
#include "opencv2/core.hpp"
#include "ros2_shared/string_printf.hpp"
#include "task_thread.hpp"
#include "tf_utils.hpp"
#include "vmap_context.hpp"

namespace fiducial_vlam
{
// ==============================================================================
// SamBuildMarkerMapTask class
// ==============================================================================

  class SamBuildMarkerMapTask
  {
    const VmapContext &cxt_;
    const Map &empty_map_;
    const gtsam::SharedNoiseModel corner_noise_;

    gtsam::ISAM2 isam_{get_isam2_params()};
    std::map<gtsam::Key, std::uint64_t> marker_seen_counts_{};

    std::uint64_t frames_processed_{0};

    static gtsam::ISAM2Params get_isam2_params()
    {
      gtsam::ISAM2Params params;
      params.factorization = gtsam::ISAM2Params::QR;
      params.relinearizeThreshold = 0.01;
      params.relinearizeSkip = 1;
//      params.evaluateNonlinearError = true;
      return params;
    }

    std::uint64_t good_marker_seen_counts_{0};
    bool good_marker_is_fixed_{false};
    const Observation *good_marker_observation_{nullptr};

    void good_marker_start()
    {
      good_marker_seen_counts_ = 0;
      good_marker_is_fixed_ = false;
      good_marker_observation_ = nullptr;
    }

    void good_marker_check(const Observation &observation)
    {
      if (good_marker_is_fixed_) {
        return;
      }

      // A fixed marker will always be the best
      auto marker_ptr = empty_map_.find_marker_const(observation.id());
      if (marker_ptr != nullptr && marker_ptr->is_fixed()) {
        good_marker_observation_ = &observation;
        good_marker_is_fixed_ = true;
        return;
      }

      // Otherwise search for the marker that has been viewed in the most frames.
      auto marker_key{GtsamUtil::marker_key(observation.id())};
      auto pair = marker_seen_counts_.find(marker_key);
      if (pair != marker_seen_counts_.end()) {
        if (pair->second > good_marker_seen_counts_) {
          good_marker_seen_counts_ = pair->second;
          good_marker_observation_ = &observation;
        }
      }
    }

    const Observation *good_marker()
    {
      return good_marker_observation_;
    }


    gtsam::Pose3 solve_camera_f_marker(
      const Observation &observation,
      const CameraInfoInterface &camera_info)
    {
      auto cv_t_camera_marker = CvUtils::solve_t_camera_marker(observation, camera_info, empty_map_.marker_length());
      return GtsamUtil::to_pose3(cv_t_camera_marker.transform().inverse());
    }


    void add_project_between_factors(const Observations &observations,
                                     const std::shared_ptr<const gtsam::Cal3DS2> &cal3ds2,
                                     gtsam::Key camera_key,
                                     const std::function<bool(bool, const Observation &)> &do_add_func,
                                     gtsam::NonlinearFactorGraph &graph)
    {
      for (auto &observation : observations) {
        auto marker_key{GtsamUtil::marker_key(observation.id())};

        // Look for the marker_seen_count record for this marker.
        auto pair = marker_seen_counts_.find(marker_key);

        // Check that we should add the measurement for this marker.
        if (do_add_func(pair != marker_seen_counts_.end(), observation)) {

          // The marker corners as seen in the image.
          auto corners_f_image = observation.to_point_vector<gtsam::Point2>();
          auto corners_f_marker = TFConvert::corners_f_marker<gtsam::Point3>(empty_map_.marker_length());

          // Add factors to the graph
          for (size_t j = 0; j < corners_f_image.size(); j += 1) {
            graph.emplace_shared<ProjectBetweenFactor>(corners_f_image[j],
                                                       corner_noise_,
                                                       marker_key,
                                                       corners_f_marker[j],
                                                       camera_key,
                                                       cal3ds2);
          }

          // update the marker seen counts
          if (pair == marker_seen_counts_.end()) {
            marker_seen_counts_.emplace(marker_key, 1);
          } else {
            pair->second += 1;
          }
        }
      }
    }

  public:
    SamBuildMarkerMapTask(const VmapContext &cxt, const Map &empty_map) :
      cxt_{cxt}, empty_map_{empty_map},
      corner_noise_{gtsam::noiseModel::Isotropic::Sigma(2, cxt_.map_corner_measurement_sigma_)}
    {
      // Initialize the isam with the fixed prior
      gtsam::NonlinearFactorGraph graph{};
      gtsam::Values initial{};

      // Add priors and initial estimates for fixed markers
      for (auto &pair : empty_map_.markers()) {
        if (pair.second.is_fixed()) {
          auto marker_key{GtsamUtil::marker_key(pair.second.id())};
          auto marker_f_map{GtsamUtil::to_pose3(pair.second.t_map_marker().transform())};

          // Add the prior
          graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
            marker_key, marker_f_map,
            gtsam::noiseModel::Constrained::MixedSigmas(gtsam::Z_6x1));

          // Add the initial estimate.
          initial.insert(marker_key, marker_f_map);

          // Update the seen counts saying we have seen this fixed marker once.
          marker_seen_counts_.emplace(marker_key, 1);
        }
      }

      isam_.update(graph, initial);
    }

    void process_observations(const Observations &observations,
                              const CameraInfoInterface &camera_info)
    {
      gttic(process_observations);
      auto camera_key{GtsamUtil::camera_key(frames_processed_)};
      bool unknown_exist{false};
//      int update1, update2, update3, update4;

      auto cal3ds2{GtsamUtil::make_cal3ds2(camera_info)};

      { // First pass through the markers for those that have been seen already
        gttic(first_update_result);
        gtsam::NonlinearFactorGraph graph{};
        gtsam::Values initial{};

        // Prepare for the best marker search. When we optimize the system given the known markers,
        // we need to have an estimate for the camera pose for this frame. This will be used as the
        // initial pose of the camera for the optimization. To find an appropriate camera pose estimate
        // we will choose one "best" marker and use it's location and measurement to estimate an
        // initial pose for the camera.
        good_marker_start();

        auto do_add_func = [this, &unknown_exist](bool known_marker, const Observation &observation) -> bool
        {
          unknown_exist = (unknown_exist || !known_marker);
          if (known_marker) {
            good_marker_check(observation);
          }
          return known_marker;
        };

        // Actually add the factors to the isam structure. The do_add_function specifies if
        // a factor is added for a particular marker. This invocation will add factors for
        // markers that have previously been seen.
        add_project_between_factors(observations, cal3ds2, camera_key, do_add_func, graph);

        // If there is no good marker (if there are no known markers) then just return.
        if (good_marker() == nullptr) {
          return;
        }

        // Get the latest estimate of the good marker location from the isam solver.
        auto good_marker_f_world = isam_.calculateEstimate<gtsam::Pose3>(GtsamUtil::marker_key(good_marker()->id()));

        // Find the camera pose relative to a good marker using the image points.
        // Calculate the good estimate of camera_f_world and set as the initial value.
        auto cv_camera_f_good_marker = solve_camera_f_marker(*good_marker(), camera_info);
        auto camera_f_world_good = good_marker_f_world * cv_camera_f_good_marker.inverse();
        initial.insert(camera_key, camera_f_world_good);


        // Update iSAM with the factors for known markers. This will find the best estimate for the
        // camera pose which is used below for calculating an estimate of new marker poses.
        gttic(update1);
        isam_.update(graph, initial);
//        std::cout << "1 "
//                  << first_update_result.errorBefore.get() << " "
//                  << first_update_result.errorAfter.get() << std::endl;
        gttoc(update1);
        gttic(update2);
        isam_.update();
//        std::cout << "  "
//                  << next_update_result.errorBefore.get() << " "
//                  << next_update_result.errorAfter.get() << std::endl;
        gttoc(update2);
        gttoc(first_update_result);
      }


      if (unknown_exist) {
        gttic(last_update_result);
        // Second pass through the markers for those that have not been seen yet
        gtsam::NonlinearFactorGraph graph{};
        gtsam::Values initial{};

        // Get the latest estimate of the camera location
        auto camera_f_world_latest = isam_.calculateEstimate<gtsam::Pose3>(camera_key);

        auto do_add_func = [this, &camera_info, &initial, camera_f_world_latest](
          bool known_marker, const Observation &observation) -> bool
        {
          if (!known_marker) {
            auto cv_camera_f_marker = solve_camera_f_marker(observation, camera_info);
            auto marker_f_world = camera_f_world_latest * cv_camera_f_marker.inverse();
            initial.insert(GtsamUtil::marker_key(observation.id()), marker_f_world);
          }
          return !known_marker;
        };

        // This time add factors for markers that haven't been previously seen.
        add_project_between_factors(observations, cal3ds2, camera_key, do_add_func, graph);

        // Update iSAM with the new factors to unknown markers
        gttic(update3);
        isam_.update(graph, initial);
        gttoc(update3);
        gttic(update4);
        isam_.update();
        gttoc(update4);
        gttoc(last_update_result);
      }

      frames_processed_ += 1;
//      std::cout << "Frame " << frames_processed_ << std::endl;
//                << "- error before:" << first_update_result.errorBefore.value()
//                << " after:" << last_update_result.errorAfter.value() << std::endl;
      gttoc(process_observations);
    }

    std::unique_ptr<Map> solve_map()
    {
      gttic(solve_map);
      auto new_map = std::make_unique<Map>(empty_map_);

      // Build up the new map by looping through all the marker_seen_counts and
      // adding them to the map.
      gtsam::Values bestEstimate = isam_.calculateBestEstimate();
      for (auto &pair : marker_seen_counts_) {
        auto marker_key{pair.first};
        auto marker_id{static_cast<int>(gtsam::Symbol{marker_key}.index())};

        // Figure if this marker already exists in the map.
        auto map_marker_ptr = new_map->find_marker(marker_id);

        // If this is a fixed marker, then we only have to update the counts.
        // Don't change the pose. (Although ISAM shouldn't have changed it because
        // of the prior factor that fixed the pose.
        if (map_marker_ptr != nullptr && map_marker_ptr->is_fixed()) {
          map_marker_ptr->set_update_count(pair.second);
          continue;
        }

        // Get the pose and covariance from the solver
        auto t_map_marker = bestEstimate.at<gtsam::Pose3>(marker_key);
        gtsam::Pose3::Jacobian t_map_marker_cov;
        try {
          t_map_marker_cov = isam_.marginalCovariance(marker_key);
        } catch (gtsam::IndeterminantLinearSystemException &ex) {
          t_map_marker_cov.diagonal().setZero();
        }

        // If this marker does not exist in the map, add it.
        if (map_marker_ptr == nullptr) {
          Marker new_marker{marker_id, GtsamUtil::to_transform_with_covariance(
            t_map_marker, t_map_marker_cov)};
          new_marker.set_update_count(pair.second);
          new_map->add_marker(new_marker);
          continue;
        }

        // If the map contains this marker as a non-fixed marker, then update the
        // existing marker in the map. (This shouldn't happen because the empty_map
        // is supposed to only contain fixed marker(s)).
        map_marker_ptr->set_t_map_marker(GtsamUtil::to_transform_with_covariance(
          t_map_marker, t_map_marker_cov));
        map_marker_ptr->set_update_count(pair.second);
      }

      gttoc(solve_map);

#ifdef ENABLE_TIMING
      gtsam::tictoc_print();
      gtsam::tictoc_reset_();
#endif

      return new_map;
    }
  };

// ==============================================================================
// SamBuildMarkerMapImpl class
// ==============================================================================

  class SamBuildMarkerMapImpl : public BuildMarkerMapInterface
  {
    // These parameters are captured when the class is constructed. This allows
    // the map creation to proceed in one mode.
    const VmapContext cxt_;
    std::unique_ptr<Map> empty_map_;

    task_thread::TaskThread<SamBuildMarkerMapTask> task_thread_;
    std::future<std::unique_ptr<Map>> solve_map_future_{};

    std::uint64_t frames_added_count_{0};
    std::uint64_t solve_map_updates_count_{0};
    bool stop_adding_observations_{false};

  public:
    SamBuildMarkerMapImpl(const VmapContext &cxt,
                          const Map &empty_map) :
      cxt_{cxt},
      empty_map_{std::make_unique<Map>(empty_map)},
      task_thread_(std::make_unique<SamBuildMarkerMapTask>(cxt_, *empty_map_), !cxt.map_compute_on_thread_)
    {}

    void process_observations(std::unique_ptr<const Observations> observations,
                              std::unique_ptr<const CameraInfoInterface> camera_info) override
    {
      // A stop command will stop adding observations.
      if (stop_adding_observations_) {
        return;
      }

      frames_added_count_ += 1;

      auto func = [obs = std::move(observations), ci = std::move(camera_info)](SamBuildMarkerMapTask &stw) -> void
      {
        stw.process_observations(*obs, *ci);
      };

      task_thread_.push(std::move(func));
    }

    std::string build_marker_map(Map &map) override
    {
      auto msg{ros2_shared::string_print::f("build_marker_map frames: added %d, processed %d",
                                            frames_added_count_,
                                            frames_added_count_ - task_thread_.tasks_in_queue())};

      // If the future is valid, then a map is being solved and we should check
      // to see if it is complete
      if (solve_map_future_.valid()) {

        // Is it complete?
        bool complete = solve_map_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready;
        if (!complete) {
          return ros2_shared::string_print::f("%s, need %d, waiting for needed frames to be processed",
                                              msg.c_str(), solve_map_updates_count_);
        }

        // If this .get() throws a "broken promise" exception the cause is likely the stw.solve_map() line
        // below. solve_map() probably throws an exception that isn't caught and prevents the set_value()
        // method from executing.
        auto new_map = solve_map_future_.get();
        if (new_map) {
          map.reset(*new_map);
        }
        return ros2_shared::string_print::f("%s, map complete with %d markers.",
                                            msg.c_str(), map.markers().size());
      }

      solve_map_updates_count_ = frames_added_count_;

      // A map is not being solved, so queue a solution up.
      std::promise<std::unique_ptr<Map>> solve_map_promise{};
      solve_map_future_ = solve_map_promise.get_future();

      auto func = [promise = std::move(solve_map_promise)](SamBuildMarkerMapTask &stw) mutable -> void
      {
        std::unique_ptr<Map> new_map{};
        try {
          new_map = stw.solve_map();
        } catch (const std::exception &e) {
          std::cout << "EXCEPTION :" << e.what() << std::endl;
        }
        promise.set_value(std::move(new_map));
      };

      task_thread_.push(std::move(func));

      return ros2_shared::string_print::f("%s, need %d, build map task queued",
                                          msg.c_str(), solve_map_updates_count_);
    }

    std::string map_cmd(std::string &cmd) override
    {
      if (cmd == "start") {
        return std::string{"SamBuildMarkerMap Start map creation."};
      }

      if (cmd == "stop") {
        stop_adding_observations_ = true;
        return std::string{"SamBuildMarkerMap no observations will be processed."};
      }

      if (cmd == "continue") {
        stop_adding_observations_ = false;
        return std::string{"SamBuildMarkerMap observations will again be processed."};
      }

      return std::string{};
    }
  };

  std::unique_ptr<BuildMarkerMapInterface> make_sam_build_marker_map(const VmapContext &cxt,
                                                                     const Map &empty_map)
  {
    return std::make_unique<SamBuildMarkerMapImpl>(cxt, empty_map);
  }

}
