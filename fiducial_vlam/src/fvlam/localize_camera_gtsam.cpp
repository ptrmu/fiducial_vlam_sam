#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"

#include "fvlam/localize_camera_interface.hpp"
#include "fvlam/camera_info.hpp"
#include "fvlam/logger.hpp"
#include "fvlam/marker.hpp"
#include "fvlam/observation.hpp"
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include "fvlam/factors_gtsam.hpp"

namespace fvlam
{

// ==============================================================================
// LocalizeCameraBetweenFactor class
// ==============================================================================

  class LocalizeCameraBetweenFactor : public LocalizeCameraInterface
  {
    LocalizeCameraProjectBetweenContext lc_context_;
    Logger &logger_;
    std::unique_ptr<LocalizeCameraInterface> lc_cv_;

    gtsam::Symbol camera_key_{'c', 0};

    void add_project_between_factors(const Observations &observations,
                                     const gtsam::Cal3DS2 &cal3ds2,
                                     const MarkerMap &map,
                                     const Transform3WithCovariance &t_map_camera_initial,
                                     gtsam::NonlinearFactorGraph &graph,
                                     gtsam::Values &initial)
    {
      auto corner_noise{gtsam::noiseModel::Isotropic::Sigma(2, lc_context_.corner_measurement_sigma_)};

      // Add the camera initial value.
      initial.insert(camera_key_, t_map_camera_initial.tf().to<gtsam::Pose3>());

      // Add measurement factors, known marker priors, and marker initial estimates to the graph
      for (auto &observation : observations.observations()) {
        gtsam::Symbol marker_key{'m', observation.id()};

        // See if this is a known marker by looking it up in the map.
        auto marker_ptr = map.find_marker_const(observation.id());

        // Add the measurement and initial value if this is a known marker.
        if (marker_ptr != nullptr) {

          // The marker corners as seen in the image.
          auto corners_f_image = observation.to<std::vector<gtsam::Point2>>();
          auto corners_f_marker = Marker::corners_f_marker<std::vector<gtsam::Point3>>(map.marker_length());

          // Add factors to the graph.
          for (size_t j = 0; j < corners_f_image.size(); j += 1) {
            graph.emplace_shared<ProjectBetweenFactor>(corners_f_image[j], corner_noise,
                                                       marker_key, camera_key_,
                                                       corners_f_marker[j], cal3ds2,
                                                       logger_, true);
          }

          // Add the marker initial value.
          auto t_world_marker = marker_ptr->t_world_marker().tf().to<gtsam::Pose3>();
          initial.insert(marker_key, t_world_marker);

          // Add the marker prior with a noise model. NOTE: using the marker covariance
          // ends up giving some unsatisfactory results. The camera location does not optimize
          // to the best answer. Not quite sure of the explanation. When the markers have uncertainty
          // they move around to find lower optimums with the camera also moving.
          bool use_constrained = marker_ptr->is_fixed() ||
                                 !lc_context_.use_marker_covariance_ ||
                                 marker_ptr->t_world_marker().cov()(0, 0) == 0.0;

          // Create the appropriate marker pose prior noise model.
          auto noise_model = use_constrained ?
                             gtsam::noiseModel::Constrained::MixedSigmas(gtsam::Z_6x1) :
                             gtsam::noiseModel::Gaussian::Covariance(
                               GtsamUtil::cov_gtsam_from_ros(
                                 marker_ptr->t_world_marker().tf().to<gtsam::Pose3>(),
                                 marker_ptr->t_world_marker().cov()));

//          auto noise_model = gtsam::noiseModel::Diagonal::Sigmas(
//            (gtsam::Vector(6) << gtsam::Vector3::Constant(0.1), gtsam::Vector3::Constant(0.3)).finished());

          // Add the prior for the known marker.
          graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3> >(marker_key,
                                                                  t_world_marker,
                                                                  noise_model);
        }
      }
    }

  public:
    LocalizeCameraBetweenFactor(const LocalizeCameraProjectBetweenContext &lc_context, Logger &logger) :
      lc_context_{lc_context}, logger_{logger},
      lc_cv_{make_localize_camera(fvlam::LocalizeCameraCvContext(), logger)}
    {
      logger_.debug() << "Construct LocalizeCameraBetweenFactor";
    }

    // Given observations of fiducial markers and a map of world locations of those
    // markers, figure out the camera pose in the world frame.
    Transform3WithCovariance solve_t_map_camera(const Observations &observations,
                                                const CameraInfo &camera_info,
                                                const MarkerMap &map) override
    {
      // Get an estimate of camera_f_map.
      auto t_map_camera_cv = lc_cv_->solve_t_map_camera(observations, camera_info, map);

      // If we could not find an estimate, then there are no known markers in the image.
      if (!t_map_camera_cv.is_valid()) {
        return t_map_camera_cv;
      }

      // Create a GTSAM camera calibration structure
      auto cal3ds2 = camera_info.to<gtsam::Cal3DS2>();


      // 1. Allocate the graph and initial estimate
      gtsam::NonlinearFactorGraph graph{};
      gtsam::Values initial{};

      // 2. add factors to the graph
      add_project_between_factors(observations, cal3ds2,
                                  map, t_map_camera_cv,
                                  graph, initial);

      // 4. Optimize the graph using Levenberg-Marquardt
      auto params = gtsam::LevenbergMarquardtParams();
      params.setRelativeErrorTol(1e-8);
      params.setAbsoluteErrorTol(1e-8);
//      params.setVerbosity("TERMINATION");

      try {
        auto result = gtsam::LevenbergMarquardtOptimizer(graph, initial, params).optimize();
//      logger_.debug() << "initial error = " << graph.error(initial) << std::endl;
//      logger_.debug() << "final error = " << graph.error(result) << std::endl;

        // 5. Extract the result into a Transform3WithCovariance
        return GtsamUtil::extract_transform3_with_covariance(graph, result, camera_key_);

      } catch (gtsam::CheiralityException &e) {
      }
      return Transform3WithCovariance{};
    }

    // Given the corners of one marker (observation) calculate t_camera_marker.
    Transform3WithCovariance solve_t_camera_marker(const Observation &observation,
                                                   const CameraInfo &camera_info,
                                                   double marker_length) override
    {
      // for now just use the CV version.
      return lc_cv_->solve_t_camera_marker(observation, camera_info, marker_length);
    }
  };

  template<>
  std::unique_ptr<LocalizeCameraInterface> make_localize_camera<LocalizeCameraProjectBetweenContext>(
    const LocalizeCameraProjectBetweenContext &lc_context, Logger &logger)
  {
    return std::make_unique<LocalizeCameraBetweenFactor>(lc_context, logger);
  }

// ==============================================================================
// LocalizeCameraResectioning class
// ==============================================================================

  class LocalizeCameraResectioning : public LocalizeCameraInterface
  {
    LocalizeCameraResectioningContext lc_context_;
    Logger &logger_;
    std::unique_ptr<LocalizeCameraInterface> lc_cv_;

    gtsam::Symbol camera_key_{'c', 0};

    void add_resectioning_factors(const Observations &observations,
                                  const gtsam::Cal3DS2 &cal3ds2,
                                  const MarkerMap &map,
                                  const Transform3WithCovariance &t_map_camera_initial,
                                  gtsam::NonlinearFactorGraph &graph,
                                  gtsam::Values &initial)
    {
      auto corner_noise{gtsam::noiseModel::Isotropic::Sigma(2, lc_context_.corner_measurement_sigma_)};

      // Add the camera initial value.
      initial.insert(camera_key_, t_map_camera_initial.tf().to<gtsam::Pose3>());

      // Add measurement factors, known marker priors, and marker initial estimates to the graph
      for (auto &observation : observations.observations()) {

        // See if this is a known marker by looking it up in the map.
        auto marker_ptr = map.find_marker_const(observation.id());

        // Add the measurement and initial value if this is a known marker.
        if (marker_ptr != nullptr) {

          // The marker corners as seen in the image.
          auto corners_f_image = observation.to<std::vector<gtsam::Point2>>();
          auto corners_f_map = marker_ptr->corners_f_world<std::vector<gtsam::Point3>>(map.marker_length());

          // Add factors to the graph.
          for (size_t j = 0; j < corners_f_image.size(); j += 1) {
            graph.emplace_shared<ResectioningFactor>(corners_f_image[j], corner_noise,
                                                     camera_key_, corners_f_map[j], cal3ds2,
                                                     logger_, true);
          }
        }
      }
    }

  public:
    LocalizeCameraResectioning(const LocalizeCameraResectioningContext &lc_context, Logger &logger) :
      lc_context_{lc_context}, logger_{logger},
      lc_cv_{make_localize_camera(fvlam::LocalizeCameraCvContext(), logger)}
    {
      logger_.debug() << "Construct LocalizeCameraResectioning";
    }

    // Given observations of fiducial markers and a map of world locations of those
    // markers, figure out the camera pose in the world frame.
    Transform3WithCovariance solve_t_map_camera(const Observations &observations,
                                                const CameraInfo &camera_info,
                                                const MarkerMap &map) override
    {
      // Get an estimate of camera_f_map.
      auto t_map_camera_cv = lc_cv_->solve_t_map_camera(observations, camera_info, map);

      // If we could not find an estimate, then there are no known markers in the image.
      if (!t_map_camera_cv.is_valid()) {
        return t_map_camera_cv;
      }

      // Create a GTSAM camera calibration structure
      auto cal3ds2 = camera_info.to<gtsam::Cal3DS2>();

      // 1. Allocate the graph and initial estimate
      gtsam::NonlinearFactorGraph graph{};
      gtsam::Values initial{};

      // 2. add factors to the graph
      add_resectioning_factors(observations, cal3ds2,
                               map, t_map_camera_cv,
                               graph, initial);

      // 4. Optimize the graph using Levenberg-Marquardt
      auto params = gtsam::LevenbergMarquardtParams();
      params.setRelativeErrorTol(1e-8);
      params.setAbsoluteErrorTol(1e-8);
//      params.setVerbosity("TERMINATION");

      try {
        auto result = gtsam::LevenbergMarquardtOptimizer(graph, initial, params).optimize();
//      logger_.debug() << "initial error = " << graph.error(initial) << std::endl;
//      logger_.debug() << "final error = " << graph.error(result) << std::endl;

        // 5. Extract the result into a Transform3WithCovariance
        return GtsamUtil::extract_transform3_with_covariance(graph, result, camera_key_);

      } catch (gtsam::CheiralityException &e) {
      }
      return Transform3WithCovariance{};
    }

    // Given the corners of one marker (observation) calculate t_camera_marker.
    Transform3WithCovariance solve_t_camera_marker(const Observation &observation,
                                                   const CameraInfo &camera_info,
                                                   double marker_length) override
    {
      // for now just use the CV version.
      return lc_cv_->solve_t_camera_marker(observation, camera_info, marker_length);
    }
  };

  template<>
  std::unique_ptr<LocalizeCameraInterface> make_localize_camera<LocalizeCameraResectioningContext>(
    const LocalizeCameraResectioningContext &lc_context, Logger &logger)
  {
    return std::make_unique<LocalizeCameraResectioning>(lc_context, logger);
  }
}
