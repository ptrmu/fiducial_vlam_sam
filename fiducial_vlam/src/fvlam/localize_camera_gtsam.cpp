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
// LocalizeCameraGtsamFactor class
// ==============================================================================

  class LocalizeCameraGtsamFactor : public LocalizeCameraInterface
  {
    LocalizeCameraGtsamFactorContext lc_context_;
    Logger &logger_;
    std::unique_ptr<LocalizeCameraInterface> lc_cv_;

    gtsam::Symbol camera_key_{'c', 0};

    void add_factors_resectioning(const Observations &observations,
                                  const CameraInfo &camera_info,
                                  const MarkerMap &map,
                                  gtsam::NonlinearFactorGraph &graph,
                                  gtsam::Values &initial)
    {
      // Get an estimate of camera_f_map.
      auto t_map_camera_initial = lc_cv_->solve_t_map_camera(observations, camera_info, map);

      // If we could not find an estimate, then there are no known markers in the image.
      if (!t_map_camera_initial.is_valid()) {
        return;
      }

      // Create a GTSAM camera calibration structure.
      auto cal3ds2 = std::make_shared<const gtsam::Cal3DS2>(camera_info.to<gtsam::Cal3DS2>());

      // Create a noise model for the corners measurements
      auto corner_noise = gtsam::noiseModel::Isotropic::Sigma(2, lc_context_.corner_measurement_sigma_);

      // Add the camera initial value.
      initial.insert(camera_key_, t_map_camera_initial.tf().to<gtsam::Pose3>());

      // Add measurement factors to the graph
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

    void add_factors_project_between(const Observations &observations,
                                     const CameraInfo &camera_info,
                                     const MarkerMap &map,
                                     gtsam::NonlinearFactorGraph &graph,
                                     gtsam::Values &initial)
    {
      // Get an estimate of camera_f_map.
      auto t_map_camera_initial = lc_cv_->solve_t_map_camera(observations, camera_info, map);

      // If we could not find an estimate, then there are no known markers in the image.
      if (!t_map_camera_initial.is_valid()) {
        return;
      }

      // Create a GTSAM camera calibration structure.
      auto cal3ds2 = std::make_shared<const gtsam::Cal3DS2>(camera_info.to<gtsam::Cal3DS2>());

      // Create a noise model for the corners measurements
      auto corner_noise = gtsam::noiseModel::Isotropic::Sigma(2, lc_context_.corner_measurement_sigma_);

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

    void add_factors_quad_resectioning(const Observations &observations,
                                       const CameraInfo &camera_info,
                                       const MarkerMap &map,
                                       gtsam::NonlinearFactorGraph &graph,
                                       gtsam::Values &initial)
    {
      // Get an estimate of camera_f_map.
      auto t_map_camera_initial = lc_cv_->solve_t_map_camera(observations, camera_info, map);

      // If we could not find an estimate, then there are no known markers in the image.
      if (!t_map_camera_initial.is_valid()) {
        return;
      }

      // Create a GTSAM camera calibration structure.
      auto cal3ds2 = std::make_shared<const gtsam::Cal3DS2>(camera_info.to<gtsam::Cal3DS2>());

      // Create a noise model for the corners measurements
      auto corner_noise = gtsam::noiseModel::Isotropic::Sigma(8, lc_context_.corner_measurement_sigma_);

      // Add the camera initial value.
      initial.insert(camera_key_, t_map_camera_initial.tf().to<gtsam::Pose3>());

      // Add measurement factors to the graph
      for (auto &observation : observations.observations()) {

        // See if this is a known marker by looking it up in the map.
        auto marker_ptr = map.find_marker_const(observation.id());

        // Add the measurement and initial value if this is a known marker.
        if (marker_ptr != nullptr) {

          // The marker corners as seen in the image.
          auto corners_f_image = observation.to<std::vector<gtsam::Point2>>();
          auto corners_f_map = marker_ptr->corners_f_world<std::vector<gtsam::Point3>>(map.marker_length());

          // Add factor to the graph.
          graph.emplace_shared<QuadResectioningFactor>(corners_f_image, corner_noise,
                                                       camera_key_, corners_f_map, cal3ds2,
                                                       logger_, true);
        }
      }
    }

    void add_factors(const Observations &observations,
                     const CameraInfo &camera_info,
                     const MarkerMap &map,
                     gtsam::NonlinearFactorGraph &graph,
                     gtsam::Values &initial)
    {
      switch (lc_context_.gtsam_factor_type_) {
        default:
        case 0:
          add_factors_resectioning(observations, camera_info, map, graph, initial);
          break;
        case 1:
          add_factors_project_between(observations, camera_info, map, graph, initial);
          break;
        case 2:
          add_factors_quad_resectioning(observations, camera_info, map, graph, initial);
          break;
      }
    }

  public:
    LocalizeCameraGtsamFactor(const LocalizeCameraGtsamFactorContext &lc_context, Logger &logger) :
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

      // 1. Allocate the graph and initial estimate
      gtsam::NonlinearFactorGraph graph{};
      gtsam::Values initial{};

      // 2. add factors to the graph
      add_factors(observations, camera_info, map, graph, initial);

      if (!initial.empty()) {

        // 4. Optimize the graph using Levenberg-Marquardt
        auto params = gtsam::LevenbergMarquardtParams();
        params.setRelativeErrorTol(1e-8);
        params.setAbsoluteErrorTol(1e-8);
//      params.setVerbosity("TERMINATION");

        try {
          auto result = gtsam::LevenbergMarquardtOptimizer(graph, initial, params).optimize();
//        logger_.debug() << "initial error = " << graph.error(initial) << std::endl;
//        logger_.debug() << "final error = " << graph.error(result) << std::endl;

          // 5. Extract the result into a Transform3WithCovariance
          return GtsamUtil::extract_transform3_with_covariance(graph, result, camera_key_);

        } catch (gtsam::CheiralityException &e) {
        }
      }

      return Transform3WithCovariance{};
    }

  };

  template<>
  std::unique_ptr<LocalizeCameraInterface> make_localize_camera<LocalizeCameraGtsamFactorContext>(
    const LocalizeCameraGtsamFactorContext &lc_context, Logger &logger)
  {
    return std::make_unique<LocalizeCameraGtsamFactor>(lc_context, logger);
  }
}
