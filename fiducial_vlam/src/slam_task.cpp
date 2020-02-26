
#include "fiducial_math.hpp"

//#define ENABLE_TIMING

#include <gtsam/base/timing.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include "gtsam/inference/Symbol.h"
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/InitializePose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include "map.hpp"
#include "observation.hpp"
#include "ros2_shared/string_printf.hpp"
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

    static unsigned char marker_key_char()
    {
      return 'm';
    }

    static gtsam::Key marker_key(std::uint64_t j)
    {
      return gtsam::Symbol{marker_key_char(), j};
    }

    static gtsam::Key camera_key(std::uint64_t j)
    {
      return gtsam::Symbol{'c', j};
    }
  };

// ==============================================================================
// ResectioningFactor class
// ==============================================================================
#if 0
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
#endif

// ==============================================================================
// ProjectBetweenFactor class
// ==============================================================================

  class ProjectBetweenFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>
  {
    const gtsam::Cal3DS2 cal3ds2_; // Make a copy. A shared object might be better.
    const gtsam::Point3 point_f_marker_;
    const gtsam::Point2 point_f_image_;

  public:
    ProjectBetweenFactor(gtsam::Point2 point_f_image,
                         const gtsam::SharedNoiseModel &model,
                         const gtsam::Key key_marker,
                         gtsam::Point3 point_f_marker,
                         const gtsam::Key key_camera,
                         const gtsam::Cal3DS2 &cal3ds2) :
      NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(model, key_marker, key_camera),
      cal3ds2_{cal3ds2},
      point_f_marker_(std::move(point_f_marker)),
      point_f_image_(std::move(point_f_image))
    {}

    /// evaluate the error
    gtsam::Vector evaluateError(const gtsam::Pose3 &marker_f_world,
                                const gtsam::Pose3 &camera_f_world,
                                boost::optional<gtsam::Matrix &> H1,
                                boost::optional<gtsam::Matrix &> H2) const override
    {
      gtsam::Matrix36 d_point3_wrt_pose3;
      gtsam::Matrix26 d_point2_wrt_pose3;
      gtsam::Matrix23 d_point2_wrt_point3;

      // Transform the point from the Marker frame to the World frame
      gtsam::Point3 point_f_world = marker_f_world.transform_from(
        point_f_marker_,
        H1 ? gtsam::OptionalJacobian<3, 6>(d_point3_wrt_pose3) : boost::none);

      // Project this point to the camera's image frame. Catch and return a default
      // value on a CheiralityException.
      auto camera = gtsam::PinholeCamera<gtsam::Cal3DS2>{camera_f_world, cal3ds2_};
      try {
        gtsam::Point2 point_f_image = camera.project(
          point_f_world,
          H2 ? gtsam::OptionalJacobian<2, 6>(d_point2_wrt_pose3) : boost::none,
          H1 ? gtsam::OptionalJacobian<2, 3>(d_point2_wrt_point3) : boost::none);

        // Return the Jacobian for each input
        if (H1) {
          *H1 = d_point2_wrt_point3 * d_point3_wrt_pose3;
        }
        if (H2) {
          *H2 = d_point2_wrt_pose3;
        }

        // Return the error.
        return point_f_image - point_f_image_;

      } catch (gtsam::CheiralityException &e) {
        std::cout << "ProjectBetweenFactor CheiralityException Exception!" << std::endl;
      }
      if (H1) *H1 = gtsam::Matrix26::Zero();
      if (H2) *H2 = gtsam::Matrix26::Zero();
      return gtsam::Vector2{2.0 * cal3ds2_.fx(), 2.0 * cal3ds2_.fy()};
    }
  };

#if 0
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
      gtsam::Values result_{};
      std::map<gtsam::Key, std::uint64_t> marker_seen_counts_{};
      std::uint64_t frames_processed_{0};
      std::uint64_t last_frames_processed_{0};
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
        auto params = gtsam::LevenbergMarquardtParams();
        params.setRelativeErrorTol(1e-8);
        params.setAbsoluteErrorTol(1e-8);
        auto result = gtsam::LevenbergMarquardtOptimizer(graph, initial, params).optimize();
  //      std::cout << "camera_f_marker initial error = " << graph.error(initial) << std::endl;
  //      std::cout << "final error = " << graph.error(result) << std::endl;

        // 5. Extract the result
        camera_f_marker = result.at<gtsam::Pose3>(resectioning_camera_key_);
        gtsam::Marginals marginals(graph, result);
        camera_f_marker_cov = marginals.marginalCovariance(resectioning_camera_key_);
      }

      void add_between_factor(
        const Observation &observation,
        const CameraInfo &camera_info,
        gtsam::Pose3 &camera_f_marker)
      {
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
            auto marker_f_map{GtsamUtil::to_pose3(pair.second.t_map_marker().transform())};

            // Add the prior
            graph_.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
              marker_key,
              marker_f_map,
              gtsam::noiseModel::Constrained::MixedSigmas(gtsam::Z_6x1));

            // Add the initial estimate.
            initial_.insert(marker_key, marker_f_map);
          }
        }
      }

      void process_observations(const Observations &observations,
                                const CameraInfo &camera_info)
      {
        // Loop through the observations and figure out which marker observation (the best)
        // should be used to figure out the camera pose.
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

        gtsam::Pose3 camera_f_marker;

        // Add the best observation to the graph.
        add_between_factor(*best_observation, camera_info, camera_f_marker);

        // Figure out the camera pose estimate for this frame. Later, use the initial camera pose estimate
        // to figure out the initial marker pose estimates for new markers.
        auto best_marker_f_map = initial_.at<gtsam::Pose3>(GtsamUtil::marker_key(best_observation->id()));
        auto t_map_camera = best_marker_f_map * camera_f_marker;

        initial_.insert(GtsamUtil::camera_key(frames_processed_), t_map_camera);

        // Use that one observation of a known marker to figure out an initial
        // estimate of the pose of the camera.
        // loop through all the observations,
        for (auto &observation : observations.observations()) {

          // Skip the best observation that has already been processed.
          if (best_observation == &observation) {
            continue;
          }

          // From the observation, figure out the pose of the camera in the
          // marker frame.
          add_between_factor(observation, camera_info, camera_f_marker);

          // If this is the first time we have seen a marker, add an initial estimate.
          auto marker_key{GtsamUtil::marker_key(observation.id())};
          if (!initial_.exists(marker_key)) {
            auto marker_f_map = t_map_camera * camera_f_marker.inverse();
            initial_.insert(marker_key, marker_f_map);
          }
        }

        frames_processed_ += 1;

        // Optimize the graph
#if 1
        auto params = gtsam::LevenbergMarquardtParams();
  //      params.setVerbosityLM("TERMINATION");
  //      params.setVerbosity("TERMINATION");
        params.setRelativeErrorTol(1e-8);
        params.setAbsoluteErrorTol(1e-8);

        result_ = gtsam::LevenbergMarquardtOptimizer(graph_, initial_, params).optimize();
        std::cout << "Frame " << frames_processed_ << ": ";
  //      std::cout << "initial error = " << graph_.error(initial_) << std::endl;
        std::cout << "final error = " << graph_.error(result_) << std::endl;
#else
        auto params = gtsam::DoglegParams();
  //      params.setVerbosityLM("TERMINATION");
  //      params.setVerbosity("TERMINATION");
        params.setRelativeErrorTol(1e-8);
        params.setAbsoluteErrorTol(1e-8);

        result_ = gtsam::DoglegOptimizer(graph_, initial_, params).optimize();
        std::cout << "Frame " << frames_processed_ << ": ";
  //      std::cout << "initial error = " << graph_.error(initial_) << std::endl;
        std::cout << "final error = " << graph_.error(result_) << std::endl;
#endif

        // Update the initial estimate with the values from the optimization.
  //      initial_ = result;
      }

      std::unique_ptr<Map> solve_map()
      {
        auto new_map = std::make_unique<Map>(empty_map_);

        // Don't bother publishing a mew map if no new frames have been processed.
        if (last_frames_processed_ == frames_processed_) {
          return new_map;
        }
        last_frames_processed_ = frames_processed_;

  //      // Optimize the graph
  //      auto params = gtsam::LevenbergMarquardtParams();
  //      params.setVerbosityLM("TERMINATION");
  //      params.setVerbosity("TERMINATION");
  //      params.setRelativeErrorTol(1e-8);
  //      params.setAbsoluteErrorTol(1e-8);
  //
  //      auto result = gtsam::LevenbergMarquardtOptimizer(graph_, initial_, params).optimize();
  //      std::cout << "Frame " << frames_processed_ << ": ";
  //      std::cout << "initial error = " << graph_.error(initial_) << std::endl;
  //      std::cout << "final error = " << graph_.error(initial_) << std::endl;

        // Build up the new map.
        for (auto &pair : marker_seen_counts_) {
          auto marker_key{pair.first};
          auto marker_id{static_cast<int>(gtsam::Symbol{marker_key}.index())};

          auto t_map_marker = GtsamUtil::extract_transform_with_covariance(graph_, result_, marker_key);

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
  // ProjectBetweenTaskWork class
  // ==============================================================================

    class ProjectBetweenTaskWork
    {
      FiducialMath &fm_;
      const FiducialMathContext &cxt_;
      const Map &empty_map_;
      const gtsam::SharedNoiseModel corner_noise_;

      gtsam::NonlinearFactorGraph graph_{};
      gtsam::Values initial_{};
      gtsam::Values result_{};
      std::map<gtsam::Key, std::uint64_t> marker_seen_counts_{};
      std::uint64_t frames_processed_{0};
      std::uint64_t last_frames_processed_{0};
      gtsam::Key resectioning_camera_key_{gtsam::Symbol('c', 0)};


      gtsam::Pose3 solve_camera_f_marker(
        const Observation &observation,
        const CameraInfo &camera_info)
      {
        auto cv_t_camera_marker = fm_.solve_t_camera_marker(observation, camera_info, empty_map_.marker_length());
        return GtsamUtil::to_pose3(cv_t_camera_marker.transform().inverse());
      }

      void add_between_factor(
        const Observation &observation,
        const CameraInfo &camera_info,
        gtsam::Pose3 &camera_f_marker)
      {
        gtsam::Pose3::Jacobian camera_f_marker_cov;

        // From the observation, figure out the pose of the camera in the
        // marker frame.
        camera_f_marker = solve_camera_f_marker(observation, camera_info);

        // Add ProjectBetween factors to the graph
        auto marker_key{GtsamUtil::marker_key(observation.id())};
        auto corners_f_image = observation.to_point_array<gtsam::Point2>();
        auto corners_f_marker = Convert::corners_f_marker<gtsam::Point3>(empty_map_.marker_length());
        for (size_t j = 0; j < corners_f_image.size(); j += 1) {
          graph_.emplace_shared<ProjectBetweenFactor>(corners_f_image[j],
                                                      corner_noise_,
                                                      marker_key,
                                                      corners_f_marker[j],
                                                      GtsamUtil::camera_key(frames_processed_),
                                                      camera_info.cal3ds2());
        }

        // Update the marker seen counts
        auto pair = marker_seen_counts_.find(marker_key);
        bool first_time = (pair == marker_seen_counts_.end());
        if (first_time) {
          marker_seen_counts_.emplace(marker_key, 1);
        } else {
          pair->second += 1;
        }
      }

    public:
      ProjectBetweenTaskWork(FiducialMath &fm, const FiducialMathContext &cxt, const Map &empty_map) :
        fm_{fm}, cxt_{cxt}, empty_map_{empty_map},
        corner_noise_{gtsam::noiseModel::Isotropic::Sigma(2, cxt_.corner_measurement_sigma_)}
      {
        // Add priors and initial estimates for fixed markers
        for (auto &pair : empty_map_.markers()) {
          if (pair.second.is_fixed()) {
            auto marker_key{GtsamUtil::marker_key(pair.second.id())};
            auto marker_f_map{GtsamUtil::to_pose3(pair.second.t_map_marker().transform())};

            // Add the prior
            graph_.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
              marker_key,
              marker_f_map,
              gtsam::noiseModel::Constrained::MixedSigmas(gtsam::Z_6x1));

            // Add the initial estimate.
            initial_.insert(marker_key, marker_f_map);
            result_.insert(marker_key, marker_f_map);
          }
        }
      }

      void process_observations(const Observations &observations,
                                const CameraInfo &camera_info)
      {
        // Loop through the observations and figure out which marker observation (the best)
        // should be used to figure out the camera pose.
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

        gtsam::Pose3 camera_f_marker;

        // Add the best observation to the graph.
        add_between_factor(*best_observation, camera_info, camera_f_marker);

        // Figure out the camera pose estimate for this frame. Later, use the initial camera pose estimate
        // to figure out the initial marker pose estimates for new markers.
        auto best_marker_f_map = result_.at<gtsam::Pose3>(GtsamUtil::marker_key(best_observation->id()));
        auto t_map_camera = best_marker_f_map * camera_f_marker;

        initial_.insert(GtsamUtil::camera_key(frames_processed_), t_map_camera);

        // Use that one observation of a known marker to figure out an initial
        // estimate of the pose of the camera.
        // loop through all the observations,
        for (auto &observation : observations.observations()) {

          // Skip the best observation that has already been processed.
          if (best_observation == &observation) {
            continue;
          }

          // From the observation, figure out the pose of the camera in the
          // marker frame.
          add_between_factor(observation, camera_info, camera_f_marker);

          // If this is the first time we have seen a marker, add an initial estimate.
          auto marker_key{GtsamUtil::marker_key(observation.id())};
          if (!initial_.exists(marker_key)) {
            auto marker_f_map = t_map_camera * camera_f_marker.inverse();
            initial_.insert(marker_key, marker_f_map);
          }
        }

        frames_processed_ += 1;

        // Optimize the graph
#if 0
        auto params = gtsam::LevenbergMarquardtParams();
  //      params.setVerbosityLM("TRYLAMBDA");
        params.setVerbosity("TERMINATION");
        params.setRelativeErrorTol(1e-8);
        params.setAbsoluteErrorTol(1e-8);

  //      graph_.print("\ngraph\n");
  //      initial_.print("\ninitial\n");

        result_ = gtsam::LevenbergMarquardtOptimizer(graph_, initial_, params).optimize();
        std::cout << "Frame " << frames_processed_ << ": ";
        std::cout << "initial error = " << graph_.error(initial_) << std::endl;
        std::cout << "final error = " << graph_.error(result_) << std::endl;
#else
        auto params = gtsam::DoglegParams();
  //      params.setVerbosityDL("VERBOSE");
        params.setVerbosity("TERMINATION");
        params.setRelativeErrorTol(1e-8);
        params.setAbsoluteErrorTol(1e-8);

        result_ = gtsam::DoglegOptimizer(graph_, initial_, params).optimize();
        std::cout << "Frame " << frames_processed_ << ": ";
  //      std::cout << "initial error = " << graph_.error(initial_) << std::endl;
        std::cout << "final error = " << graph_.error(result_) << std::endl;
#endif

        // Update the initial estimate with the values from the optimization.
        initial_ = result_;
      }

      std::unique_ptr<Map> solve_map()
      {
        auto new_map = std::make_unique<Map>(empty_map_);

        // Don't bother publishing a mew map if no new frames have been processed.
        if (last_frames_processed_ == frames_processed_) {
          return new_map;
        }
        last_frames_processed_ = frames_processed_;

  //      // Optimize the graph
  //      auto params = gtsam::LevenbergMarquardtParams();
  //      params.setVerbosityLM("TERMINATION");
  //      params.setVerbosity("TERMINATION");
  //      params.setRelativeErrorTol(1e-8);
  //      params.setAbsoluteErrorTol(1e-8);
  //
  //      auto result = gtsam::LevenbergMarquardtOptimizer(graph_, initial_, params).optimize();
  //      std::cout << "Frame " << frames_processed_ << ": ";
  //      std::cout << "initial error = " << graph_.error(initial_) << std::endl;
  //      std::cout << "final error = " << graph_.error(initial_) << std::endl;

        // Build up the new map.
        for (auto &pair : marker_seen_counts_) {
          auto marker_key{pair.first};
          auto marker_id{static_cast<int>(gtsam::Symbol{marker_key}.index())};

          auto t_map_marker = GtsamUtil::extract_transform_with_covariance(graph_, result_, marker_key);

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
#endif
#if 0
// ==============================================================================
// TaskWork class
// ==============================================================================

  class TaskWork
  {
    FiducialMath &fm_;
    const FiducialMathContext &cxt_;
    const Map &empty_map_;
    const gtsam::SharedNoiseModel corner_noise_;

    gtsam::ISAM2 isam_{get_isam2_params()};
    std::map<gtsam::Key, std::uint64_t> marker_seen_counts_{};

    std::uint64_t frames_processed_{0};
    std::uint64_t last_frames_processed_{0};

    static gtsam::ISAM2Params get_isam2_params()
    {
      gtsam::ISAM2Params params;
      params.factorization = gtsam::ISAM2Params::QR;
      params.relinearizeThreshold = 0.01;
      params.relinearizeSkip = 1;
      params.evaluateNonlinearError = true;
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
      const CameraInfo &camera_info)
    {
      auto cv_t_camera_marker = fm_.solve_t_camera_marker(observation, camera_info, empty_map_.marker_length());
      return GtsamUtil::to_pose3(cv_t_camera_marker.transform().inverse());
    }


    void add_project_between_factors(const Observations &observations,
                                     const CameraInfo &camera_info,
                                     gtsam::Key camera_key,
                                     std::function<bool(bool, const Observation &)> do_add_func,
                                     gtsam::NonlinearFactorGraph &graph)
    {
      for (auto &observation : observations.observations()) {
        auto marker_key{GtsamUtil::marker_key(observation.id())};

        // Look for the marker_seen_count record for this marker.
        auto pair = marker_seen_counts_.find(marker_key);

        // Check that we should add the measurement for this marker.
        if (do_add_func(pair != marker_seen_counts_.end(), observation)) {

          // The marker corners as seen in the image.
          auto corners_f_image = observation.to_point_vector<gtsam::Point2>();
          auto corners_f_marker = Convert::corners_f_marker<gtsam::Point3>(empty_map_.marker_length());

          // Add factors to the graph
          for (size_t j = 0; j < corners_f_image.size(); j += 1) {
            graph.emplace_shared<ProjectBetweenFactor>(corners_f_image[j],
                                                       corner_noise_,
                                                       marker_key,
                                                       corners_f_marker[j],
                                                       camera_key,
                                                       camera_info.cal3ds2());
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
    TaskWork(FiducialMath &fm, const FiducialMathContext &cxt, const Map &empty_map) :
      fm_{fm}, cxt_{cxt}, empty_map_{empty_map},
      corner_noise_{gtsam::noiseModel::Isotropic::Sigma(2, cxt_.corner_measurement_sigma_)}
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
                              const CameraInfo &camera_info)
    {
      gttic(process_observations);
      auto camera_key{GtsamUtil::camera_key(frames_processed_)};
      bool unknown_exist{false};
      gtsam::ISAM2Result first_update_result;
      gtsam::ISAM2Result last_update_result;
      int update1, update2, update3, update4;

      { // First pass through the markers for those that have been seen already
        gttic(first_update_result);
        gtsam::NonlinearFactorGraph graph{};
        gtsam::Values initial{};

        // Prepare for the best marker search.
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
        add_project_between_factors(observations, camera_info, camera_key, do_add_func, graph);

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
        first_update_result = isam_.update(graph, initial);
        std::cout << "1 "
                  << first_update_result.errorBefore.get() << " "
                  << first_update_result.errorAfter.get() << std::endl;
        gttoc(update1);
        gttic(update2);
        auto next_update_result = isam_.update();
        std::cout << "  "
                  << next_update_result.errorBefore.get() << " "
                  << next_update_result.errorAfter.get() << std::endl;
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

        auto do_add_func = [this, camera_info, &initial, camera_f_world_latest](
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
        add_project_between_factors(observations, camera_info, camera_key, do_add_func, graph);

        // Update iSAM with the new factors to unknown markers
        gttic(update3);
        isam_.update(graph, initial);
        gttoc(update3);
        gttic(update4);
        last_update_result = isam_.update();
        gttoc(update4);
        gttoc(last_update_result);
      }

      frames_processed_ += 1;
      std::cout << "Frame " << frames_processed_ << std::endl;
//                << "- error before:" << first_update_result.errorBefore.value()
//                << " after:" << last_update_result.errorAfter.value() << std::endl;
      gttoc(process_observations);
    }

    std::unique_ptr<Map> solve_map()
    {
      gttic(solve_map);
      auto new_map = std::make_unique<Map>(empty_map_);

      // Don't bother publishing a mew map if no new frames have been processed.
      if (last_frames_processed_ == frames_processed_) {
        return new_map;
      }
      last_frames_processed_ = frames_processed_;


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
// SlamTask class
// ==============================================================================

#define USE_THREAD

  class SlamTask : public UpdateMapInterface
  {
    FiducialMath &fm_;
    // These parameters are captured when the class is constructed. This allows
    // the map creation to proceed in one mode.
    const FiducialMathContext cxt_;
    std::unique_ptr<Map> empty_map_;
    std::future<std::unique_ptr<Map>> solve_map_future_{};

    task_thread::TaskThread<TaskWork> task_thread_;
    std::unique_ptr<TaskWork> stw_; // This will ultimately get owned by the thread

    std::uint64_t frames_added_count_{0};
    std::uint64_t solve_map_updates_count_{0};

  public:
    SlamTask(FiducialMath &fm, const FiducialMathContext &cxt, const Map &empty_map) :
      UpdateMapInterface{}, fm_{fm}, cxt_{cxt},
      empty_map_{std::make_unique<Map>(empty_map)},
      task_thread_(std::make_unique<TaskWork>(fm, cxt_, *empty_map_)),
      stw_{std::make_unique<TaskWork>(fm, cxt_, *empty_map_)}
    {}

    ~SlamTask() = default;

    void update_map(const Observations &observations,
                    const CameraInfo &camera_info,
                    Map &map) override
    {
      frames_added_count_ += 1;

      auto func = [observations, camera_info](TaskWork &stw) -> void
      {
        stw.process_observations(observations, camera_info);
      };
#ifdef USE_THREAD
      task_thread_.push(func);
#else
      func(*stw_);
#endif
    }

    void update_map_for_publishing(Map &map) override
    {
      auto msg{
        ros2_shared::string_print::f("update_map_for_publishing queued tasks:%d", task_thread_.tasks_in_queue())};

      // If the future is valid, then a map is being solved and we should check
      // to see if it is complete
      if (solve_map_future_.valid()) {

        // Is it complete?
        auto status = solve_map_future_.wait_for(std::chrono::milliseconds(0));
        if (status == std::future_status::ready) {
          auto new_map = solve_map_future_.get();
          map.reset(*new_map);
        }
        std::cout << ros2_shared::string_print::f("%s, solve_map queued:%d, status=%s\n",
                                                  msg.c_str(), solve_map_updates_count_,
                                                  (status == std::future_status::ready) ? "DONE!!!!!!" : "waiting");
        return;
      }

      solve_map_updates_count_ = frames_added_count_;

      // A map is not being solved, so queue a solution up.
      std::promise<std::unique_ptr<Map>> solve_map_promise{};
      solve_map_future_ = solve_map_promise.get_future();

      auto func = [promise = std::move(solve_map_promise)](TaskWork &stw) mutable -> void
      {
        auto new_map = stw.solve_map();
        promise.set_value(std::move(new_map));
      };

#ifdef USE_THREAD
      task_thread_.push(std::move(func));
#else
      func(*stw_);
#endif

      std::cout << ros2_shared::string_print::f("%s, update queued\n",
                                                msg.c_str());

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
#endif

// ==============================================================================
// SamBuildMarkerMapTask class
// ==============================================================================

  class SamBuildMarkerMapTask
  {
    CvFiducialMathInterface &fm_;
    const FiducialMathContext &cxt_;
    const Map &empty_map_;
    const gtsam::SharedNoiseModel corner_noise_;

    gtsam::ISAM2 isam_{get_isam2_params()};
    std::map<gtsam::Key, std::uint64_t> marker_seen_counts_{};

    std::uint64_t frames_processed_{0};
    std::uint64_t last_frames_processed_{0};

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
      const CameraInfo &camera_info)
    {
      auto cv_t_camera_marker = fm_.solve_t_camera_marker(observation, camera_info, empty_map_.marker_length());
      return GtsamUtil::to_pose3(cv_t_camera_marker.transform().inverse());
    }


    void add_project_between_factors(const Observations &observations,
                                     const CameraInfo &camera_info,
                                     gtsam::Key camera_key,
                                     std::function<bool(bool, const Observation &)> do_add_func,
                                     gtsam::NonlinearFactorGraph &graph)
    {
      for (auto &observation : observations.observations()) {
        auto marker_key{GtsamUtil::marker_key(observation.id())};

        // Look for the marker_seen_count record for this marker.
        auto pair = marker_seen_counts_.find(marker_key);

        // Check that we should add the measurement for this marker.
        if (do_add_func(pair != marker_seen_counts_.end(), observation)) {

          // The marker corners as seen in the image.
          auto corners_f_image = observation.to_point_vector<gtsam::Point2>();
          auto corners_f_marker = Convert::corners_f_marker<gtsam::Point3>(empty_map_.marker_length());

          // Add factors to the graph
          for (size_t j = 0; j < corners_f_image.size(); j += 1) {
            graph.emplace_shared<ProjectBetweenFactor>(corners_f_image[j],
                                                       corner_noise_,
                                                       marker_key,
                                                       corners_f_marker[j],
                                                       camera_key,
                                                       camera_info.cal3ds2());
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
    SamBuildMarkerMapTask(CvFiducialMathInterface &fm, const FiducialMathContext &cxt, const Map &empty_map) :
      fm_{fm}, cxt_{cxt}, empty_map_{empty_map},
      corner_noise_{gtsam::noiseModel::Isotropic::Sigma(2, cxt_.corner_measurement_sigma_)}
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
                              const CameraInfo &camera_info)
    {
      gttic(process_observations);
      auto camera_key{GtsamUtil::camera_key(frames_processed_)};
      bool unknown_exist{false};
      gtsam::ISAM2Result first_update_result;
      gtsam::ISAM2Result last_update_result;
      int update1, update2, update3, update4;

      { // First pass through the markers for those that have been seen already
        gttic(first_update_result);
        gtsam::NonlinearFactorGraph graph{};
        gtsam::Values initial{};

        // Prepare for the best marker search.
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
        add_project_between_factors(observations, camera_info, camera_key, do_add_func, graph);

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
        first_update_result = isam_.update(graph, initial);
//        std::cout << "1 "
//                  << first_update_result.errorBefore.get() << " "
//                  << first_update_result.errorAfter.get() << std::endl;
        gttoc(update1);
        gttic(update2);
        auto next_update_result = isam_.update();
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

        auto do_add_func = [this, camera_info, &initial, camera_f_world_latest](
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
        add_project_between_factors(observations, camera_info, camera_key, do_add_func, graph);

        // Update iSAM with the new factors to unknown markers
        gttic(update3);
        isam_.update(graph, initial);
        gttoc(update3);
        gttic(update4);
        last_update_result = isam_.update();
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

      // Don't bother publishing a mew map if no new frames have been processed.
      if (last_frames_processed_ == frames_processed_) {
        return new_map;
      }
      last_frames_processed_ = frames_processed_;


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
    CvFiducialMathInterface &fm_;
    // These parameters are captured when the class is constructed. This allows
    // the map creation to proceed in one mode.
    const FiducialMathContext cxt_;
    std::unique_ptr<Map> empty_map_;

    std::unique_ptr<SamBuildMarkerMapTask> stw_; // This will ultimately get owned by the thread
    std::unique_ptr<task_thread::TaskThread<SamBuildMarkerMapTask>> task_thread_{};
    std::future<std::unique_ptr<Map>> solve_map_future_{};

    std::uint64_t frames_added_count_{0};
    std::uint64_t solve_map_updates_count_{0};

  public:
    SamBuildMarkerMapImpl(CvFiducialMathInterface &fm,
                          const FiducialMathContext &cxt,
                          const Map &empty_map) :
      fm_{fm}, cxt_{cxt},
      empty_map_{std::make_unique<Map>(empty_map)},
      stw_{std::make_unique<SamBuildMarkerMapTask>(fm, cxt_, *empty_map_)}
    {
      // If we are using a thread, create it and pass the work object to it.
      if (cxt.compute_on_thread_) {
        task_thread_ = std::make_unique<task_thread::TaskThread<SamBuildMarkerMapTask>>(std::move(stw_));
      }
    }

    virtual ~SamBuildMarkerMapImpl() = default;

    void process_observations(const Observations &observations,
                              const CameraInfo &camera_info) override
    {
      frames_added_count_ += 1;

      auto func = [observations, camera_info](SamBuildMarkerMapTask &stw) -> void
      {
        stw.process_observations(observations, camera_info);
      };

      if (task_thread_) {
        task_thread_->push(std::move(func));
      } else {
        func(*stw_);
      }
    }

    virtual std::string update_map(Map &map)
    {
      auto msg{ros2_shared::string_print::f("update_map frames: added %d, processed %d",
                                            frames_added_count_,
                                            frames_added_count_ - task_thread_->tasks_in_queue())};

      // If the future is valid, then a map is being solved and we should check
      // to see if it is complete
      if (solve_map_future_.valid()) {

        // Is it complete?
        bool complete = solve_map_future_.wait_for(std::chrono::milliseconds(0) ) == std::future_status::ready;
        if (!complete) {
          return ros2_shared::string_print::f("%s, needed %d, waiting for needed frames to be processed",
                                              msg.c_str(), solve_map_updates_count_);
        }

        auto new_map = solve_map_future_.get();
        map.reset(*new_map);
        return ros2_shared::string_print::f("%s, map complete and being published",
                                            msg.c_str());
      }

      solve_map_updates_count_ = frames_added_count_;

      // A map is not being solved, so queue a solution up.
      std::promise<std::unique_ptr<Map>> solve_map_promise{};
      solve_map_future_ = solve_map_promise.get_future();

      auto func = [promise = std::move(solve_map_promise)](SamBuildMarkerMapTask &stw) mutable -> void
      {
        auto new_map = stw.solve_map();
        promise.set_value(std::move(new_map));
      };

      if (task_thread_) {
        task_thread_->push(std::move(func));
      } else {
        func(*stw_);
      }

      return ros2_shared::string_print::f("%s, needed %d, build map task queued",
                                          msg.c_str(), solve_map_updates_count_);
    }

    virtual std::string build_marker_map_cmd(std::string &cmd)
    {
      if (cmd == "start") {
        return std::string{"SamBuildMarkerMap Start map creation."};
      }
      return std::string{};
    }
  };

  std::unique_ptr<BuildMarkerMapInterface> sam_build_marker_map_factory(CvFiducialMathInterface &fm,
                                                                        const FiducialMathContext &cxt,
                                                                        const Map &empty_map)
  {
    return std::unique_ptr<BuildMarkerMapInterface>{new SamBuildMarkerMapImpl{fm, cxt, empty_map}};
  }

}
