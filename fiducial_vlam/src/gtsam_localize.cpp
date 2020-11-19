
#include "fiducial_math.hpp"

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
#include "vloc_context.hpp"

namespace fiducial_vlam
{
// ==============================================================================
// SamLocalizeCameraImpl class
// ==============================================================================

  class SamLocalizeCameraImpl : public LocalizeCameraInterface
  {
    const VlocContext &cxt_;
    LocalizeCameraInterface &cv_lc_;

    gtsam::Key camera_key_{GtsamUtil::camera_key(0)};

  public:
    explicit SamLocalizeCameraImpl(const VlocContext &cxt, LocalizeCameraInterface &cv_lc) :
      cxt_{cxt}, cv_lc_{cv_lc}
    {}

    void add_project_between_factors(const Observations &observations, const CameraInfoInterface &camera_info,
                                     const Map &map, const TransformWithCovariance &t_map_camera_initial,
                                     gtsam::NonlinearFactorGraph &graph, gtsam::Values &initial)
    {
      auto cal3ds2{GtsamUtil::make_cal3ds2(camera_info)};
      auto corner_noise{gtsam::noiseModel::Isotropic::Sigma(2, cxt_.loc_corner_measurement_sigma_)};

      // Add the camera initial value.
      initial.insert(camera_key_, GtsamUtil::to_pose3(t_map_camera_initial.transform()));

      // Add measurement factors, known marker priors, and marker initial estimates to the graph
      for (auto &observation : observations.observations()) {
        auto marker_key{GtsamUtil::marker_key(observation.id())};

        // See if this is a known marker by looking it up in the map.
        auto marker_ptr = map.find_marker_const(observation.id());

        // Add the measurement and initial value if this is a known marker.
        if (marker_ptr != nullptr) {

          // The marker corners as seen in the image.
          auto corners_f_image = observation.to_point_vector<gtsam::Point2>();
          auto corners_f_marker = TFConvert::corners_f_marker<gtsam::Point3>(map.marker_length());

          // Add factors to the graph.
          for (size_t j = 0; j < corners_f_image.size(); j += 1) {
            graph.emplace_shared<ProjectBetweenFactor>(corners_f_image[j],
                                                       corner_noise,
                                                       marker_key,
                                                       corners_f_marker[j],
                                                       camera_key_,
                                                       cal3ds2);
          }

          // Add initial value.
          auto marker_f_map_initial{GtsamUtil::to_pose3(marker_ptr->t_map_marker().transform())};
          initial.insert(marker_key, marker_f_map_initial);

          // Add the prior.
          auto known_marker_cov{GtsamUtil::to_pose_cov_sam(marker_ptr->t_map_marker().cov())};

          bool use_constrained = marker_ptr->is_fixed() ||
                                 map.map_style() == Map::MapStyles::pose ||
                                 known_marker_cov(0, 0) == 0.0;

          // Create the appropriate marker pose prior noise model.
          auto noise_model = use_constrained ?
                             gtsam::noiseModel::Constrained::MixedSigmas(gtsam::Z_6x1) :
                             gtsam::noiseModel::Gaussian::Covariance(known_marker_cov);

          // Add the prior for the known marker.
          graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3> >(marker_key,
            marker_f_map_initial,
            noise_model);
        }
      }
    }

    TransformWithCovariance solve_t_map_camera(const Observations &observations,
                                               const CameraInfoInterface &camera_info,
                                               const Map &map) override
    {
      // Get an estimate of camera_f_map.
      auto cv_t_map_camera = cv_lc_.solve_t_map_camera(observations, camera_info, map);

      // If we could not find an estimate, then there are no known markers in the image.
      if (!cv_t_map_camera.is_valid()) {
        return cv_t_map_camera;
      }

      // 1. Allocate the graph and initial estimate
      gtsam::NonlinearFactorGraph graph{};
      gtsam::Values initial{};

      // 2. add factors to the graph
      add_project_between_factors(observations, camera_info,
                                  map, cv_t_map_camera,
                                  graph, initial);

      // 4. Optimize the graph using Levenberg-Marquardt
      auto params = gtsam::LevenbergMarquardtParams();
      params.setRelativeErrorTol(1e-8);
      params.setAbsoluteErrorTol(1e-8);
      auto result = gtsam::LevenbergMarquardtOptimizer(graph, initial, params).optimize();
      //      std::cout << "initial error = " << graph.error(initial) << std::endl;
      //      std::cout << "final error = " << graph.error(result) << std::endl;

      // 5. Extract the result
      auto t_map_camera = GtsamUtil::extract_transform_with_covariance(graph, result, camera_key_);

      // 6. Rotate the covariance into the world frame
      return t_map_camera.is_valid() ? GtsamUtil::to_cov_f_world(t_map_camera) : TransformWithCovariance{};
    }
  };

  std::unique_ptr<LocalizeCameraInterface> make_sam_localize_camera(const VlocContext &cxt,
                                                                    LocalizeCameraInterface &cv_lc)
  {
    return std::make_unique<SamLocalizeCameraImpl>(cxt, cv_lc);
  }

}
