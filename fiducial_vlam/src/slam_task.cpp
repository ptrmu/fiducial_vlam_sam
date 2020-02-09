
#include "fiducial_math.hpp"

#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include "gtsam/inference/Symbol.h"
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/InitializePose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include "map.hpp"
#include "observation.hpp"
#include "task_thread.hpp"

namespace fiducial_vlam
{
// ==============================================================================
// Convert class
// ==============================================================================

  struct Convert
  {
    template<class TPoint>
    static std::vector<TPoint> corners_f_marker(double marker_length)
    {
      return std::vector<TPoint>{
        TPoint{-marker_length / 2.f, marker_length / 2.f, 0.f},
        TPoint{marker_length / 2.f, marker_length / 2.f, 0.f},
        TPoint{marker_length / 2.f, -marker_length / 2.f, 0.f},
        TPoint{-marker_length / 2.f, -marker_length / 2.f, 0.f}};
    }

    template<class TPoint>
    static TPoint to_point(const Vector3WithCovariance &v3wc)
    {
      return TPoint{v3wc.vector3().x(),
                    v3wc.vector3().y(),
                    v3wc.vector3().z()};
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
  };

// ==============================================================================
// GtsamUtil class
// ==============================================================================

  struct GtsamUtil
  {
    static gtsam::Pose3 to_pose3(const tf2::Transform &transform)
    {
      auto q = transform.getRotation();
      auto t = transform.getOrigin();
      return gtsam::Pose3{gtsam::Rot3{q.w(), q.x(), q.y(), q.z()},
                          gtsam::Vector3{t.x(), t.y(), t.z()}};
    }

    static TransformWithCovariance::cov_type to_pose_cov_type(const gtsam::Matrix6 &cov_sam)
    {
      // Convert covariance
      TransformWithCovariance::cov_type cov;
      for (int r = 0; r < 6; r += 1) {
        for (int c = 0; c < 6; c += 1) {
          static int ro[] = {3, 4, 5, 0, 1, 2};
          cov[r * 6 + c] = cov_sam(ro[r], ro[c]);
        }
      }
      return cov;
    }

    static TransformWithCovariance to_transform_with_covariance(const gtsam::Pose3 &pose_sam,
                                                                const gtsam::Matrix6 &cov_sam)
    {
      auto q1 = pose_sam.rotation().toQuaternion().coeffs();
      auto &t = pose_sam.translation();
      return TransformWithCovariance{
        tf2::Transform{tf2::Quaternion{q1[0], q1[1], q1[2], q1[3]},
                       tf2::Vector3{t.x(), t.y(), t.z()}},
        to_pose_cov_type(cov_sam)};
    }

    static TransformWithCovariance extract_transform_with_covariance(const gtsam::NonlinearFactorGraph &graph,
                                                                     const gtsam::Values &result,
                                                                     gtsam::Key key)
    {
      gtsam::Marginals marginals(graph, result);
      return to_transform_with_covariance(result.at<gtsam::Pose3>(key),
                                          marginals.marginalCovariance(key));
    }

    static gtsam::Key marker_key(std::uint64_t j)
    {
      return gtsam::Symbol{'m', j};
    }

    static gtsam::Key camera_key(std::uint64_t j)
    {
      return gtsam::Symbol{'c', j};
    }
  };

// ==============================================================================
// ResectioningFactor class
// ==============================================================================

  class ResectioningFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3>
  {
    const gtsam::Cal3DS2 &cal3ds2_;
    const gtsam::Point3 P_;
    const gtsam::Point2 p_;

  public:
    /// Construct factor given known point P and its projection p
    ResectioningFactor(const gtsam::SharedNoiseModel &model,
                       const gtsam::Key key,
                       const gtsam::Cal3DS2 &cal3ds2,
                       gtsam::Point2 p,
                       gtsam::Point3 P) :
      NoiseModelFactor1<gtsam::Pose3>(model, key),
      cal3ds2_{cal3ds2},
      P_(std::move(P)),
      p_(std::move(p))
    {}

    /// evaluate the error
    gtsam::Vector evaluateError(const gtsam::Pose3 &pose,
                                boost::optional<gtsam::Matrix &> H) const override
    {
      auto camera = gtsam::PinholeCamera<gtsam::Cal3DS2>{pose, cal3ds2_};
      return camera.project(P_, H) - p_;
    }
  };


// ==============================================================================
// SlamTaskWork class
// ==============================================================================

  class SlamTaskWork
  {
    FiducialMath &fm_;
    const FiducialMathContext &cxt_;
    const Map &empty_map_;
    const gtsam::SharedNoiseModel corner_noise_;

    gtsam::NonlinearFactorGraph graph_{};
    std::map<gtsam::Key, std::uint64_t> marker_seen_counts_{};
    bool fixed_marker_seen_{false};
    std::uint64_t frames_processed_{0};
    gtsam::Key resectioning_camera_key_{gtsam::Symbol('c', 0)};

    void solve_camera_f_marker(
      const Observation &observation,
      const CameraInfo &camera_info,
      gtsam::Pose3 &camera_f_marker,
      gtsam::Pose3::Jacobian &camera_f_marker_cov)
    {
      // 1. Allocate the graph and initial estimate
      gtsam::NonlinearFactorGraph graph{};
      gtsam::Values initial{};

      // 2. add factors to the graph
      auto corners_f_marker = Convert::corners_f_marker<gtsam::Point3>(empty_map_.marker_length());
      auto corners_f_image = observation.to_point_array<gtsam::Point2>();
      for (size_t j = 0; j < corners_f_marker.size(); j += 1) {
        graph.emplace_shared<ResectioningFactor>(
          corner_noise_,
          resectioning_camera_key_,
          camera_info.cal3ds2(),
          corners_f_image[j],
          corners_f_marker[j]);
      }

      // 3. Add the initial estimate for the camera pose in the marker frame.
      // For now we use OpenCV to get the initial pose estimate.
      auto cv_t_camera_marker = fm_.solve_t_camera_marker(observation, camera_info, empty_map_.marker_length());
      auto camera_f_marker_initial = GtsamUtil::to_pose3(cv_t_camera_marker.transform().inverse());
      initial.insert(resectioning_camera_key_, camera_f_marker_initial);

      // 4. Optimize the graph using Levenberg-Marquardt
      auto result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();
//      std::cout << "camera_f_marker initial error = " << graph.error(initial) << std::endl;
//      std::cout << "final error = " << graph.error(result) << std::endl;

      // 5. Extract the result
      camera_f_marker = result.at<gtsam::Pose3>(resectioning_camera_key_);
//      camera_f_marker.print("\ncamera_f_marker ");
      gtsam::Marginals marginals(graph, result);
      camera_f_marker_cov = marginals.marginalCovariance(resectioning_camera_key_);
    }

  public:
    SlamTaskWork(FiducialMath &fm, const FiducialMathContext &cxt, const Map &empty_map) :
      fm_{fm}, cxt_{cxt}, empty_map_{empty_map},
      corner_noise_{gtsam::noiseModel::Isotropic::Sigma(2, cxt_.corner_measurement_sigma_)}
    {}

    void process_observations(const Observations &observations,
                              const CameraInfo &camera_info)
    {
      // loop through all the observations,
      for (auto &observation : observations.observations()) {
        gtsam::Pose3 camera_f_marker;
        gtsam::Pose3::Jacobian camera_f_marker_cov;

        // From the observation, figure out the pose of the camera in the
        // marker frame.
        solve_camera_f_marker(observation, camera_info,
                              camera_f_marker, camera_f_marker_cov);

        // Add the measurement factor.
        auto marker_key{GtsamUtil::marker_key(observation.id())};
        graph_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
          marker_key,
          GtsamUtil::camera_key(frames_processed_),
          camera_f_marker,
          gtsam::noiseModel::Gaussian::Covariance(camera_f_marker_cov * 4.));

        // Update the marker seen counts
        auto pair = marker_seen_counts_.find(marker_key);
        bool first_time = (pair == marker_seen_counts_.end());
        if (first_time) {
          marker_seen_counts_.emplace(marker_key, 1);
        } else {
          pair->second += 1;
        }

        // If this is the first time we have seen a fixed marker, then add a
        // prior to pin its pose down.
        if (first_time) {
          auto marker_ptr = empty_map_.find_marker_const(observation.id());
          if (marker_ptr != nullptr && marker_ptr->is_fixed()) {

            fixed_marker_seen_ = true;
            graph_.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
              marker_key,
              GtsamUtil::to_pose3(marker_ptr->t_map_marker().transform()),
              gtsam::noiseModel::Constrained::MixedSigmas(gtsam::Z_6x1));
          }
        }
      }

      frames_processed_ += 1;
    }

    std::unique_ptr<Map> solve_map()
    {
      auto new_map = std::make_unique<Map>(empty_map_);

      if (!fixed_marker_seen_) {
        return new_map;
      }

      // Find some appropriate initial values
      auto initial = gtsam::InitializePose3::initialize(graph_);

//      initial.print("initial\n");

      // Optimize the graph
      auto params = gtsam::LevenbergMarquardtParams();
      params.setVerbosityLM("TERMINATION");
      params.setVerbosity("TERMINATION");
      params.setRelativeErrorTol(1e-8);
      params.setAbsoluteErrorTol(1e-8);

      auto result = gtsam::LevenbergMarquardtOptimizer(graph_, initial, params).optimize();
      std::cout << "Frame " << frames_processed_ << ": " << std::endl;
      std::cout << "initial error = " << graph_.error(initial) << std::endl;
      std::cout << "final error = " << graph_.error(result) << std::endl;

      // Build up the new map.
      for (auto &pair : marker_seen_counts_) {
        auto marker_key{pair.first};
        auto marker_id{static_cast<int>(gtsam::Symbol{marker_key}.index())};

        auto t_map_marker = GtsamUtil::extract_transform_with_covariance(graph_, result, marker_key);

        // update an existing marker or add a new one.
        auto marker_ptr = new_map->find_marker(marker_id);
        if (marker_ptr == nullptr) {
          Marker new_marker{marker_id, t_map_marker};
          new_marker.set_update_count(pair.second);
          new_map->add_marker(new_marker);
        } else if (!marker_ptr->is_fixed()) {
          marker_ptr->set_t_map_marker(t_map_marker);
          marker_ptr->set_update_count(pair.second);
        }
      }

      return new_map;
    }
  };

// ==============================================================================
// BatchSlamTaskWork class
// ==============================================================================

  class BatchSlamTaskWork
  {
    FiducialMath &fm_;
    const FiducialMathContext &cxt_;
    const Map &empty_map_;
    const gtsam::SharedNoiseModel corner_noise_;

    gtsam::NonlinearFactorGraph graph_{};
    gtsam::Values initial_{};
    std::map<gtsam::Key, std::uint64_t> marker_seen_counts_{};
    bool fixed_marker_seen_{false};
    std::uint64_t frames_processed_{0};
    gtsam::Key resectioning_camera_key_{gtsam::Symbol('c', 0)};

    void solve_camera_f_marker(
      const Observation &observation,
      const CameraInfo &camera_info,
      gtsam::Pose3 &camera_f_marker,
      gtsam::Pose3::Jacobian &camera_f_marker_cov)
    {
      // 1. Allocate the graph and initial estimate
      gtsam::NonlinearFactorGraph graph{};
      gtsam::Values initial{};

      // 2. add factors to the graph
      auto corners_f_marker = Convert::corners_f_marker<gtsam::Point3>(empty_map_.marker_length());
      auto corners_f_image = observation.to_point_array<gtsam::Point2>();
      for (size_t j = 0; j < corners_f_marker.size(); j += 1) {
        graph.emplace_shared<ResectioningFactor>(
          corner_noise_,
          resectioning_camera_key_,
          camera_info.cal3ds2(),
          corners_f_image[j],
          corners_f_marker[j]);
      }

      // 3. Add the initial estimate for the camera pose in the marker frame.
      // For now we use OpenCV to get the initial pose estimate.
      auto cv_t_camera_marker = fm_.solve_t_camera_marker(observation, camera_info, empty_map_.marker_length());
      auto camera_f_marker_initial = GtsamUtil::to_pose3(cv_t_camera_marker.transform().inverse());
      initial.insert(resectioning_camera_key_, camera_f_marker_initial);

      // 4. Optimize the graph using Levenberg-Marquardt
      auto result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();
//      std::cout << "camera_f_marker initial error = " << graph.error(initial) << std::endl;
//      std::cout << "final error = " << graph.error(result) << std::endl;

      // 5. Extract the result
      camera_f_marker = result.at<gtsam::Pose3>(resectioning_camera_key_);
      gtsam::Marginals marginals(graph, result);
      camera_f_marker_cov = marginals.marginalCovariance(resectioning_camera_key_);
    }

  public:
    BatchSlamTaskWork(FiducialMath &fm, const FiducialMathContext &cxt, const Map &empty_map) :
      fm_{fm}, cxt_{cxt}, empty_map_{empty_map},
      corner_noise_{gtsam::noiseModel::Isotropic::Sigma(2, cxt_.corner_measurement_sigma_)}
    {
      // Add priors and initial estimates for fixed markers
      for (auto &pair : empty_map_.markers()) {
        if (pair.second.is_fixed()) {
          auto marker_key{GtsamUtil::marker_key(pair.second.id())};
          auto marker_f_world{GtsamUtil::to_pose3(pair.second.t_map_marker().transform())};

          // Add the prior
          graph_.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
            marker_key,
            marker_f_world,
            gtsam::noiseModel::Constrained::MixedSigmas(gtsam::Z_6x1));

          // Add the initial estimate.
          initial_.insert(marker_key, marker_f_world);
//
//          // Update the marker seen count.
//          marker_seen_counts_.emplace(marker_key, UINT64_MAX);
        }
      }
    }

    void process_observations(const Observations &observations,
                              const CameraInfo &camera_info)
    {
      // Loop through the observations and figure out which marker observation
      // should be used to figure out where the camera is.
      const Observation *best_observation = nullptr;
      std::uint64_t max_counts = 0;
      for (auto &observation : observations.observations()) {
        // A fixed marker will always be the best
        auto marker_ptr = empty_map_.find_marker_const(observation.id());
        if (marker_ptr != nullptr && marker_ptr->is_fixed()) {
          best_observation = &observation;
          break;
        }
        // Otherwise search for the marker that has been viewed in the most frames.
        auto marker_key{GtsamUtil::marker_key(observation.id())};
        auto pair = marker_seen_counts_.find(marker_key);
        if (pair != marker_seen_counts_.end()) {
          if (pair->second > max_counts) {
            max_counts = pair->second;
            best_observation = &observation;
          }
        }
      }

      // If we didn't find a marker, then we have no way to process these
      // observations so just return.
      if (best_observation == nullptr) {
        return;
      }

      // Use that one observation of a known marker to figure out an initial
      // estimate of the pose of the camera.
      // loop through all the observations,
      for (auto &observation : observations.observations()) {
        gtsam::Pose3 camera_f_marker;
        gtsam::Pose3::Jacobian camera_f_marker_cov;

        // From the observation, figure out the pose of the camera in the
        // marker frame.
        solve_camera_f_marker(observation, camera_info,
                              camera_f_marker, camera_f_marker_cov);

        // Add the measurement factor.
        auto marker_key{GtsamUtil::marker_key(observation.id())};
        graph_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
          marker_key,
          GtsamUtil::camera_key(frames_processed_),
          camera_f_marker,
          gtsam::noiseModel::Gaussian::Covariance(camera_f_marker_cov * 4.));

        // Update the marker seen counts
        auto pair = marker_seen_counts_.find(marker_key);
        bool first_time = (pair == marker_seen_counts_.end());
        if (first_time) {
          marker_seen_counts_.emplace(marker_key, 1);
        } else {
          pair->second += 1;
        }

        // If this is the first time we have seen a fixed marker, then add a
        // prior to pin its pose down.
        if (first_time) {
          auto marker_ptr = empty_map_.find_marker_const(observation.id());
          if (marker_ptr != nullptr && marker_ptr->is_fixed()) {

            fixed_marker_seen_ = true;
            graph_.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
              marker_key,
              GtsamUtil::to_pose3(marker_ptr->t_map_marker().transform()),
              gtsam::noiseModel::Constrained::MixedSigmas(gtsam::Z_6x1));
          }
        }
      }

      frames_processed_ += 1;
    }

    std::unique_ptr<Map> solve_map()
    {
      auto new_map = std::make_unique<Map>(empty_map_);

      if (!fixed_marker_seen_) {
        return new_map;
      }

      // Find some appropriate initial values
      auto initial = gtsam::InitializePose3::initialize(graph_);

//      initial.print("initial\n");

      // Optimize the graph
      auto params = gtsam::LevenbergMarquardtParams();
      params.setVerbosityLM("TERMINATION");
      params.setVerbosity("TERMINATION");
      params.setRelativeErrorTol(1e-8);
      params.setAbsoluteErrorTol(1e-8);

      auto result = gtsam::LevenbergMarquardtOptimizer(graph_, initial, params).optimize();
      std::cout << "Frame " << frames_processed_ << ": " << std::endl;
      std::cout << "initial error = " << graph_.error(initial) << std::endl;
      std::cout << "final error = " << graph_.error(result) << std::endl;

      // Build up the new map.
      for (auto &pair : marker_seen_counts_) {
        auto marker_key{pair.first};
        auto marker_id{static_cast<int>(gtsam::Symbol{marker_key}.index())};

        auto t_map_marker = GtsamUtil::extract_transform_with_covariance(graph_, result, marker_key);

        // update an existing marker or add a new one.
        auto marker_ptr = new_map->find_marker(marker_id);
        if (marker_ptr == nullptr) {
          Marker new_marker{marker_id, t_map_marker};
          new_marker.set_update_count(pair.second);
          new_map->add_marker(new_marker);
        } else if (!marker_ptr->is_fixed()) {
          marker_ptr->set_t_map_marker(t_map_marker);
          marker_ptr->set_update_count(pair.second);
        }
      }

      return new_map;
    }
  };

// ==============================================================================
// SlamTask class
// ==============================================================================

  class SlamTask : public UpdateMapInterface
  {
    FiducialMath &fm_;
    // These parameters are captured when the class is constructed. This allows
    // the map creation to proceed in one mode.
    const FiducialMathContext cxt_;
    std::unique_ptr<Map> empty_map_;
    std::future<std::unique_ptr<Map>> solve_map_future_{};

    std::unique_ptr<BatchSlamTaskWork> stw_; // This will ultimately get owned by the thread

  public:
    SlamTask(FiducialMath &fm, const FiducialMathContext &cxt, const Map &empty_map) :
      UpdateMapInterface{}, fm_{fm}, cxt_{cxt},
      empty_map_{std::make_unique<Map>(empty_map)},
      stw_{std::make_unique<BatchSlamTaskWork>(fm, cxt_, *empty_map_)}
    {}

    ~SlamTask() = default;

    void update_map(const Observations &observations,
                    const CameraInfo &camera_info,
                    Map &map) override
    {
      auto func = [observations, camera_info](BatchSlamTaskWork &stw) -> void
      {
        stw.process_observations(observations, camera_info);
      };
      func(*stw_);
    }

    void update_map_for_publishing(Map &map) override
    {
      // If the future is valid, then a map is being solved and we should check
      // to see if it is complete
      if (solve_map_future_.valid()) {

        // Is it complete?
        auto status = solve_map_future_.wait_for(std::chrono::milliseconds(0));
        if (status == std::future_status::ready) {
          auto new_map = solve_map_future_.get();
          map.reset(*new_map);
        }
        return;
      }

      // A map is not being solved, so queue a solution up.
      std::promise<std::unique_ptr<Map>> solve_map_promise{};
      solve_map_future_ = solve_map_promise.get_future();

      auto func = [promise = std::move(solve_map_promise)](BatchSlamTaskWork &stw) mutable -> void
      {
        auto new_map = stw.solve_map();
        promise.set_value(std::move(new_map));
      };

      func(*stw_);
    }

    std::string update_map_cmd(std::string &cmd) override
    {
      if (cmd == "start") {
        return std::string{"SlamTask Start map creation"};
      }
      return std::string{};
    }
  };

  std::unique_ptr<UpdateMapInterface> slam_task_factory(FiducialMath &fm,
                                                        const FiducialMathContext &cxt,
                                                        const Map &empty_map)
  {
    return std::unique_ptr<UpdateMapInterface>{new SlamTask{fm, cxt, empty_map}};
  }
}
