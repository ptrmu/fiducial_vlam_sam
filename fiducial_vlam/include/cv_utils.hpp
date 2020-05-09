
#ifndef _CV_UTILS_HPP
#define _CV_UTILS_HPP

#include "opencv2/core/types.hpp"

namespace tf2
{
  class Transform;
}

namespace fiducial_vlam
{
  class TransformWithCovariance; //
  class CameraInfoInterface; //
  class Observation;

  class Observations;

// ==============================================================================
// CvUtils class
// ==============================================================================

  struct CvUtils
  {
    static void to_cv_rvec_tvec(const TransformWithCovariance &t, cv::Vec3d &rvec, cv::Vec3d &tvec);

    static tf2::Transform to_tf2_transform(const cv::Vec3d &rvec, const cv::Vec3d &tvec);

    static Observations to_observations(const std::vector<int> &ids,
                                        const std::vector<std::vector<cv::Point2f>> &corners);

    static TransformWithCovariance solve_t_camera_marker(const Observation &observation,
                                                         const CameraInfoInterface &camera_info,
                                                         double marker_length);

  };

// ==============================================================================
// AnnotateImages class
// ==============================================================================

  struct AnnotateImages
  {
    static void with_marker_axis(cv::Mat &color_image,
                                 const TransformWithCovariance &t_camera_marker,
                                 const CameraInfoInterface &camera_info);

    static void with_detected_markers(cv::Mat &color_image,
                                      std::vector<std::vector<cv::Point2f>> aruco_corners,
                                      std::vector<int> aruco_ids,
                                      cv::Scalar borderColor = cv::Scalar(0, 255, 0));

    static void with_detected_junction(cv::Mat &color_image,
                                       const cv::Point2f &junction_f_image,
                                       const cv::Size &win_size,
                                       cv::Scalar junction_color = cv::Scalar(255, 0, 0));

    static void with_board_corners(cv::Mat &color_image,
                                   const std::array<cv::Point2f, 4> &board_corners,
                                   cv::Scalar borderColor = cv::Scalar(0, 0, 255));
  };

}
#endif //_CV_UTILS_HPP
