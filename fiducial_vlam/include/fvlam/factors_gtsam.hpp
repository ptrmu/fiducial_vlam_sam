#pragma once
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"
#pragma ide diagnostic ignored "OCUnusedStructInspection"
#pragma ide diagnostic ignored "modernize-use-nodiscard"

#include "fvlam/logger.hpp"
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

namespace fvlam
{

// ==============================================================================
// GtsamUtil class
// ==============================================================================

  struct GtsamUtil
  {
    // ROS2 has the following definition of a pose transform:
    //  # Row-major representation of the 6x6 covariance matrix (ie values in a row are stored adjacently)
    //  # The orientation parameters use a fixed-axis representation.
    //  # In order, the parameters are:
    //  # (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
    //  float64[36] covariance
    // GTSAM has the following definition of a pose transform
    //  (as per f dellaert https://groups.google.com/g/gtsam-users/c/jpz10n7VHqU/m/3IzzrhuQBAAJ)
    //  In GTSAM, a pose is a (Rot3,Point3) pair, in that order. To convert from ROS [TT, TR; RT, RR] you just
    //  form [RR, RT; TR, TT], where each is a 3*3 block.
    //
    // These covariance definitions still do not specify enough to convert covariances between ROS2 and GTSAM.
    // I did a number of experiments where I would generate a covariance in gtsam, convert it to a PoseWithCovariance
    // ROS2 message, and display it in rviz2. This is what I think I figured out:
    //  1) In GTSAM the covariance is expressed in the body frame of the t_world_body transform while in ROS2
    //    the rotation part of the covariance is in the body frame but the translation part is in the world frame.
    //  2) Both ROS2 and GTSAM use rotations about the three (x, y, z) axes to represent a 3D rotation but ROS
    //    seems to use fixed/static axes while GTSAM uses relative axes. One switches the order of the variables
    //    to generate equivalent rotations in the two systems.
    static gtsam::Pose3::Jacobian cov_gtsam_from_ros(const gtsam::Pose3 &pose_gtsam,
                                                     const Transform3::CovarianceMatrix &cov_ros)
    {
      gtsam::Pose3::Jacobian cov_gtsam_f_world;
      for (int r = 0; r < 6; r += 1) {
        for (int c = 0; c < 6; c += 1) {
          static int ro[] = {5, 4, 3, 0, 1, 2};
          cov_gtsam_f_world(r, c) = cov_ros(ro[r], ro[c]);
        }
      }

      // Rotate the translation part of the covariance from the body frame to the world frame.
      gtsam::Pose3::Jacobian rotate_translate_part;
      rotate_translate_part << gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3, pose_gtsam.rotation().matrix();
      return rotate_translate_part.transpose() * cov_gtsam_f_world * rotate_translate_part;
    }

    static Transform3::CovarianceMatrix cov_ros_from_gtsam(const gtsam::Pose3 &pose_gtsam,
                                                           const gtsam::Pose3::Jacobian &cov_gtsam)
    {
      // Rotate the translation part of the covariance from the body frame to the world frame.
      gtsam::Pose3::Jacobian rotate_translate_part;
      rotate_translate_part << gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3, pose_gtsam.rotation().matrix();
      gtsam::Pose3::Jacobian cov_gtsam_f_world = rotate_translate_part * cov_gtsam * rotate_translate_part.transpose();

      // interchange the rotate and translation variables and reverse the order of the rotation variables.
      Transform3::CovarianceMatrix cov_ros;
      for (int r = 0; r < 6; r += 1) {
        for (int c = 0; c < 6; c += 1) {
          static int ro[] = {3, 4, 5, 2, 1, 0};
          cov_ros(r, c) = cov_gtsam_f_world(ro[r], ro[c]);
        }
      }
      return cov_ros;
    }

    static Transform3WithCovariance extract_transform3_with_covariance(const gtsam::NonlinearFactorGraph &graph,
                                                                       const gtsam::Values &result,
                                                                       gtsam::Key key)
    {
      gtsam::Marginals marginals{};
      try {
        marginals = gtsam::Marginals{graph, result};
      } catch (gtsam::IndeterminantLinearSystemException &ex) {
        try {
          marginals = gtsam::Marginals{graph, result, gtsam::Marginals::QR};
        } catch (gtsam::IndeterminantLinearSystemException &ex) {
          return Transform3WithCovariance{};
        }
      }

      auto pose = result.at<gtsam::Pose3>(key);
      auto cov = static_cast<gtsam::Matrix6>(marginals.marginalCovariance(key));

      return Transform3WithCovariance{Transform3::from(pose), cov_ros_from_gtsam(pose, cov)};
    }

  };

// ==============================================================================
// ProjectBetweenFactor class
// ==============================================================================

  class ProjectBetweenFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>
  {
    gtsam::Point2 point_f_image_;
    gtsam::Key key_marker_;
    gtsam::Key key_camera_;
    gtsam::Point3 point_f_marker_;
    const gtsam::Cal3DS2 &cal3ds2_;
    Logger &logger_;
    bool throwCheirality_;     // If true, rethrows Cheirality exceptions (default: false)

  public:
    ProjectBetweenFactor(gtsam::Point2 point_f_image,
                         const gtsam::SharedNoiseModel &model,
                         gtsam::Key key_marker,
                         gtsam::Key key_camera,
                         gtsam::Point3 point_f_marker,
                         const gtsam::Cal3DS2 &cal3ds2,
                         Logger &logger,
                         bool throwCheirality = false) :
      NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(model, key_marker, key_camera),
      point_f_image_{std::move(point_f_image)},
      key_marker_{key_marker}, key_camera_{key_camera},
      point_f_marker_{std::move(point_f_marker)},
      cal3ds2_{cal3ds2},
      logger_{logger},
      throwCheirality_{throwCheirality}
    {}

    /// evaluate the error
    gtsam::Vector evaluateError(const gtsam::Pose3 &marker_f_world,
                                const gtsam::Pose3 &camera_f_world,
                                boost::optional<gtsam::Matrix &> H1 = boost::none,
                                boost::optional<gtsam::Matrix &> H2 = boost::none) const override
    {
      gtsam::Matrix36 d_point3_wrt_pose3;
      gtsam::Matrix26 d_point2_wrt_pose3;
      gtsam::Matrix23 d_point2_wrt_point3;

      // Transform the point from the Marker frame to the World frame
      gtsam::Point3 point_f_world = marker_f_world.transformFrom(
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
        if (H1) *H1 = gtsam::Matrix26::Zero();
        if (H2) *H2 = gtsam::Matrix26::Zero();

        logger_.error() << e.what() << ": Marker " << gtsam::DefaultKeyFormatter(key_marker_) <<
                        " moved behind camera " << gtsam::DefaultKeyFormatter(key_camera_) << std::endl;

        if (throwCheirality_)
          throw gtsam::CheiralityException(key_camera_);
      }
      return gtsam::Vector2{2.0 * cal3ds2_.px(), 2.0 * cal3ds2_.py()};
    }
  };

// ==============================================================================
// ProjectBetweenFactor class
// ==============================================================================


  class ResectioningFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3>
  {
    const gtsam::Point2 point_f_image;
    gtsam::Key key_camera_;
    const gtsam::Point3 point_f_world;
    const gtsam::Cal3DS2 &cal3ds2_;
    Logger &logger_;
    bool throwCheirality_;     // If true, rethrows Cheirality exceptions (default: false)

  public:
    /// Construct factor given known point P and its projection p
    ResectioningFactor(gtsam::Point2 point_f_image,
                       const gtsam::SharedNoiseModel &model,
                       gtsam::Key key_camera,
                       gtsam::Point3 point_f_world,
                       const gtsam::Cal3DS2 &cal3ds2,
                       Logger &logger,
                       bool throwCheirality = false) :
      NoiseModelFactor1<gtsam::Pose3>(model, key_camera),
      point_f_image{std::move(point_f_image)},
      key_camera_{key_camera},
      point_f_world{std::move(point_f_world)},
      cal3ds2_{cal3ds2},
      logger_{logger},
      throwCheirality_{throwCheirality}
    {}

    /// evaluate the error
    gtsam::Vector evaluateError(const gtsam::Pose3 &pose,
                                boost::optional<gtsam::Matrix &> H = boost::none) const override
    {
      auto camera = gtsam::PinholeCamera<gtsam::Cal3DS2>{pose, cal3ds2_};
      try {
        return camera.project(point_f_world, H) - point_f_image;
      } catch (gtsam::CheiralityException &e) {
        if (H) *H = gtsam::Matrix26::Zero();

        logger_.error() << e.what() << ": point moved behind camera "
                        << gtsam::DefaultKeyFormatter(key_camera_) << std::endl;

        if (throwCheirality_)
          throw gtsam::CheiralityException(key_camera_);
      }
      return gtsam::Vector2{2.0 * cal3ds2_.px(), 2.0 * cal3ds2_.py()};
    }
  };

}
