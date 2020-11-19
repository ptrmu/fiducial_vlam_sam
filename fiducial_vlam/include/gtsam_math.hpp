#ifndef FIDUCIAL_VLAM_GTSAM_MATH_HPP
#define FIDUCIAL_VLAM_GTSAM_MATH_HPP


#include "opencv2/core/types.hpp"
#include "ros2_shared/param_macros.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "transform_with_covariance.hpp"

namespace fiducial_vlam
{
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

    static gtsam::Matrix6 to_pose_cov_sam(const TransformWithCovariance::cov_type cov)
    {
      gtsam::Matrix6 cov_sam;
      for (int r = 0; r < 6; r += 1) {
        for (int c = 0; c < 6; c += 1) {
          static int ro[] = {3, 4, 5, 0, 1, 2};
          cov_sam(ro[r], ro[c]) = cov[r * 6 + c];
        }
      }
      return cov_sam;
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
      try {
        auto marginals{gtsam::Marginals{graph, result}};
        return to_transform_with_covariance(result.at<gtsam::Pose3>(key),
                                            marginals.marginalCovariance(key));

      } catch (gtsam::IndeterminantLinearSystemException &ex) {
        try {
          auto marginals{gtsam::Marginals{graph, result, gtsam::Marginals::QR}};
          return to_transform_with_covariance(result.at<gtsam::Pose3>(key),
                                              marginals.marginalCovariance(key));
        } catch (gtsam::IndeterminantLinearSystemException &ex) {
        }
      }

      return TransformWithCovariance{};
    }

    static TransformWithCovariance to_cov_f_world(const TransformWithCovariance &twc)
    {
      auto pose_sam = to_pose3(twc.transform());
      auto cov_sam = to_pose_cov_sam(twc.cov());

      // Rotate the covariance from the body frame to the world frame.
      gtsam::Matrix6 adjoint_map = pose_sam.AdjointMap();
      gtsam::Matrix6 cov_f_world = adjoint_map * cov_sam * adjoint_map.transpose();

      return to_transform_with_covariance(pose_sam, cov_f_world);
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

    static std::shared_ptr<const gtsam::Cal3DS2> make_cal3ds2(const CameraInfoInterface &camera_info)
    {
      auto &cm{camera_info.camera_matrix()};
      auto &dc{camera_info.dist_coeffs()};
      return std::make_shared<const gtsam::Cal3DS2>(cm(0, 0),  // fx
                                                    cm(1, 1),  // fy
                                                    1.0, // s
                                                    cm(0, 2),  // u0
                                                    cm(1, 2),  // v0
                                                    dc(0), // k1
                                                    dc(1), // k2
                                                    dc(2), // p1
                                                    dc(3));// p2
    }
  };

// ==============================================================================
// ProjectBetweenFactor class
// ==============================================================================

  class ProjectBetweenFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>
  {
    std::shared_ptr<const gtsam::Cal3DS2> cal3ds2_; // hold on to a shared pointer.
    const gtsam::Point3 point_f_marker_;
    const gtsam::Point2 point_f_image_;

  public:
    ProjectBetweenFactor(gtsam::Point2 point_f_image,
                         const gtsam::SharedNoiseModel &model,
                         const gtsam::Key key_marker,
                         gtsam::Point3 point_f_marker,
                         const gtsam::Key key_camera,
                         const std::shared_ptr<const gtsam::Cal3DS2> &cal3ds2) :
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

//      marker_f_world.print("\nmarker_f_world\n");
//      camera_f_world.print("\ncamera_f_world\n");
//      cal3ds2_->print("\ncal3ds2\n");
//      point_f_world.print("point_f_world\n");

      // Project this point to the camera's image frame. Catch and return a default
      // value on a CheiralityException.
      auto camera = gtsam::PinholeCamera<gtsam::Cal3DS2>{camera_f_world, *cal3ds2_};
      try {
        gtsam::Point2 point_f_image = camera.project(
          point_f_world,
          H2 ? gtsam::OptionalJacobian<2, 6>(d_point2_wrt_pose3) : boost::none,
          H1 ? gtsam::OptionalJacobian<2, 3>(d_point2_wrt_point3) : boost::none);

//        point_f_image.print("point_f_image\n");

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
      return gtsam::Vector2{2.0 * cal3ds2_->px(), 2.0 * cal3ds2_->py()};
    }
  };

}

#endif //FIDUCIAL_VLAM_GTSAM_MATH_HPP
