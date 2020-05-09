
#include "cv_utils.hpp"

#include "fiducial_math.hpp"
#include "observation.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "tf_utils.hpp"
#include "transform_with_covariance.hpp"

namespace fiducial_vlam
{

// ==============================================================================
// CvUtils class
// ==============================================================================

  void CvUtils::to_cv_rvec_tvec(const TransformWithCovariance &t,
                                cv::Vec3d &rvec, cv::Vec3d &tvec)
  {
    auto c = t.transform().getOrigin();
    tvec[0] = c.x();
    tvec[1] = c.y();
    tvec[2] = c.z();
    auto R = t.transform().getBasis();
    cv::Mat rmat(3, 3, CV_64FC1);
    for (int row = 0; row < 3; row++) {
      for (int col = 0; col < 3; col++) {
        rmat.at<double>(row, col) = R[row][col];
      }
    }
    cv::Rodrigues(rmat, rvec);
  }

  tf2::Transform CvUtils::to_tf2_transform(const cv::Vec3d &rvec, const cv::Vec3d &tvec)
  {
    cv::Mat rmat;
    cv::Rodrigues(rvec, rmat);
    tf2::Matrix3x3 m;
    for (int row = 0; row < 3; row++) {
      for (int col = 0; col < 3; col++) {
        m[row][col] = rmat.at<double>(row, col);  // Row- vs. column-major order
      }
    }
    return tf2::Transform(m, tf2::Vector3{tvec[0], tvec[1], tvec[2]});
  }


   Observations CvUtils::to_observations(const std::vector<int> &ids,
                                      const std::vector<std::vector<cv::Point2f>> &corners)
  {
    Observations observations;
    for (int i = 0; i < ids.size(); i += 1) {
      observations.add(Observation(ids[i],
                                   corners[i][0].x, corners[i][0].y,
                                   corners[i][1].x, corners[i][1].y,
                                   corners[i][2].x, corners[i][2].y,
                                   corners[i][3].x, corners[i][3].y));
    }
    return observations;
  }

  TransformWithCovariance CvUtils::solve_t_camera_marker(const Observation &observation,
                                                         const CameraInfoInterface &camera_info,
                                                         double marker_length)
  {
    // Build up two lists of corner points: 2D in the image frame, 3D in the marker frame
    auto corners_f_marker{TFConvert::corners_f_marker<cv::Point3d>(marker_length)};
    auto corners_f_image{observation.to_point_vector<cv::Point2d>()};

    // Figure out marker pose.
    cv::Vec3d rvec, tvec;
    cv::solvePnP(corners_f_marker, corners_f_image,
                 camera_info.camera_matrix(), camera_info.dist_coeffs(),
                 rvec, tvec);

    // rvec, tvec output from solvePnp "brings points from the model coordinate system to the
    // camera coordinate system". In this case the marker frame is the model coordinate system.
    // So rvec, tvec are the transformation t_camera_marker.
    return TransformWithCovariance(CvUtils::to_tf2_transform(rvec, tvec));
  }

// ==============================================================================
// AnnotateImages class
// ==============================================================================

  void AnnotateImages::with_marker_axis(cv::Mat &color_marked,
                                        const TransformWithCovariance &t_camera_marker,
                                        const CameraInfoInterface &camera_info)
  {
    cv::Vec3d rvec, tvec;
    CvUtils::to_cv_rvec_tvec(t_camera_marker, rvec, tvec);

    cv::aruco::drawAxis(color_marked,
                        camera_info.camera_matrix(), camera_info.dist_coeffs(),
                        rvec, tvec, 0.1);
  }

  void AnnotateImages::with_detected_markers(cv::Mat &color_image,
                                             std::vector<std::vector<cv::Point2f>> aruco_corners,
                                             std::vector<int> aruco_ids,
                                             cv::Scalar borderColor)
  {
    // calculate colors
    cv::Scalar textColor, cornerColor;
    textColor = cornerColor = borderColor;
    cv::swap(textColor.val[0], textColor.val[1]);     // text color just sawp G and R
    cv::swap(cornerColor.val[1], cornerColor.val[2]); // corner color just sawp G and B

    std::swap(textColor.val[0], textColor.val[1]);     // text color just sawp G and R
    std::swap(cornerColor.val[1], cornerColor.val[2]); // corner color just sawp G and B

    int nMarkers = static_cast<int>(aruco_corners.size());
    for (int i = 0; i < nMarkers; i++) {

      auto &currentMarker = aruco_corners[i];

      // draw marker sides
      for (int j = 0; j < 4; j++) {
        auto p0 = currentMarker[j];
        auto p1 = currentMarker[(j + 1) % 4];
        cv::line(color_image, p0, p1, borderColor, 1);
      }

      // draw first corner mark
      cv::rectangle(color_image,
                    currentMarker[0] - cv::Point2f(3, 3),
                    currentMarker[0] + cv::Point2f(3, 3),
                    cornerColor, 1, cv::LINE_AA);

      // draw ID
//      if (ids.total() != 0) {
//        cv::Point2f cent(0, 0);
//        for (int p = 0; p < 4; p++)
//          cent += currentMarker.ptr<cv::Point2f>(0)[p];
//
//        cent = cent / 4.;
//        std::stringstream s;
//        s << "id=" << ids.getMat().ptr<int>(0)[i];
//        putText(color_image, s.str(), cent, cv::FONT_HERSHEY_SIMPLEX, 0.5, textColor, 2);
//      }
    }
  }


  void AnnotateImages::with_detected_junction(cv::Mat &color_image,
                                              const cv::Point2f &junction_f_image,
                                              const cv::Size &win_size,
                                              cv::Scalar junction_color)
  {
    auto j_f_image = cv::Point2f{std::round(junction_f_image.x), std::round(junction_f_image.y)};
    cv::rectangle(color_image,
                  j_f_image - cv::Point2f(3, 3),
                  j_f_image + cv::Point2f(3, 3),
                  junction_color, 1, cv::LINE_4);

    cv::rectangle(color_image,
                  j_f_image - cv::Point2f(win_size.width, win_size.height),
                  j_f_image + cv::Point2f(win_size.width, win_size.height),
                  junction_color / 2, 1, cv::LINE_4);
  }

  void AnnotateImages::with_board_corners(cv::Mat &color_image,
                                          const std::array<cv::Point2f, 4> &board_corners,
                                          cv::Scalar borderColor)
  {
    for (int j = 0; j < 4; j++) {
      auto p0 = board_corners[j];
      auto p1 = board_corners[(j + 1) % 4];
      cv::line(color_image, p0, p1, borderColor, 1);
    }
  }
}
