
#include "fvlam/camera_info.hpp"
#include "fvlam/localize_camera_interface.hpp"
#include "fvlam/marker.hpp"
#include "fvlam/observation.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc.hpp"

namespace fvlam
{

// ==============================================================================
// LocalizeCameraCv class
// ==============================================================================

  class LocalizeCameraCv : public LocalizeCameraInterface
  {
    LocalizeCameraCvContext lc_context_;
    Logger &logger_;

  public:
    LocalizeCameraCv(const LocalizeCameraCvContext &lc_context, Logger &logger) :
      lc_context_{lc_context}, logger_{logger}
    {}

    // Given observations of fiducial markers and a map of world locations of those
    // markers, figure out the camera pose in the world frame.
    Transform3WithCovariance solve_t_map_camera(const Observations &observations,
                                                const CameraInfo &camera_info,
                                                const MarkerMap &map) override
    {
      // Build up two lists of corner points: 2D in the image frame, 3D in the map/world frame
      std::vector<cv::Point3d> all_corners_f_map;
      std::vector<cv::Point2d> all_corners_f_image;

      for (const auto &observation : observations.observations()) {
        auto marker_ptr = map.find_marker_const(observation.id());
        if (marker_ptr != nullptr) {
          marker_ptr->to_corners_f_world(map.marker_length(), all_corners_f_map);
          observation.to(all_corners_f_image);
        }
      }

      // If there are no known markers in the observation set, then don't
      // try to find the camera position
      if (all_corners_f_map.empty()) {
        return Transform3WithCovariance{};
      }

      // Figure out camera location.
      cv::Vec3d rvec, tvec;
      try {
        auto cc = camera_info.to<CvCameraCalibration>();
        cv::solvePnP(all_corners_f_map, all_corners_f_image,
                     cc.first, cc.second,
                     rvec, tvec);

        // For certain cases, there is a chance that the multi marker solvePnP will
        // return the mirror of the correct solution. So try solvePn[Ransac as well.
        if (all_corners_f_image.size() > 1 * 4 && all_corners_f_image.size() < 4 * 4) {
          cv::Vec3d rvecRansac, tvecRansac;
          cv::solvePnPRansac(all_corners_f_map, all_corners_f_image,
                             cc.first, cc.second,
                             rvecRansac, tvecRansac);

          // If the pose returned from the ransac version is very different from
          // that returned from the normal version, then use the ransac results.
          // solvePnp can sometimes pick up the wrong solution (a mirror solution).
          // solvePnpRansac does a better job in that case. But solvePnp does a
          // better job smoothing out image noise so it is prefered when it works.
          if (std::abs(rvec[0] - rvecRansac[0]) > 0.5 ||
              std::abs(rvec[1] - rvecRansac[1]) > 0.5 ||
              std::abs(rvec[2] - rvecRansac[2]) > 0.5) {
            rvec = rvecRansac;
            tvec = tvecRansac;
          }
        }
      } catch (cv::Exception &ex) {
        return Transform3WithCovariance{};
      }

//      if (tvec[0] < 0) { // specific tests for bad pose determination
//        int xxx = 9;
//      }

      // rvec, tvec output from solvePnp "brings points from the model coordinate system to the
      // camera coordinate system". In this case the map frame is the model coordinate system.
      // So rvec, tvec are the transformation t_camera_map.
      return Transform3WithCovariance(Transform3(Rotate3::from(rvec), Translate3::from(tvec)));
    }

    // Given the corners of one marker (observation) calculate t_camera_marker.
    Transform3WithCovariance solve_t_camera_marker(const Observation &observation,
                                                   const CameraInfo &camera_info,
                                                   double marker_length) override
    {
      auto cv_camera_info = camera_info.to<CvCameraCalibration>();
      auto solve_function = Marker::solve_t_camera_marker(cv_camera_info, marker_length);
      return solve_function(observation).t_world_marker();
    }
  };

  template<>
  std::unique_ptr<LocalizeCameraInterface> make_localize_camera<LocalizeCameraCvContext>(
    const LocalizeCameraCvContext &lc_context, Logger &logger)
  {
    return std::make_unique<LocalizeCameraCv>(lc_context, logger);
  }

// ==============================================================================
// FiducialMarkerCv class
// ==============================================================================


  class FiducialMarkerCv : public FiducialMarkerInterface
  {
    FiducialMarkerCvContext fm_context_;
    Logger &logger_;
    cv::Ptr<cv::aruco::Dictionary> localization_aruco_dictionary_;

  public:
    FiducialMarkerCv(const FiducialMarkerCvContext &fm_context, Logger &logger) :
      fm_context_{fm_context}, logger_{logger},
      localization_aruco_dictionary_{cv::aruco::getPredefinedDictionary(
        cv::aruco::PREDEFINED_DICTIONARY_NAME(fm_context.aruco_dictionary_id_))}
    {}

    // Look for fiducial markers in a gray image.
    Observations detect_markers(cv::Mat &gray_image) override
    {
      auto detectorParameters = cv::aruco::DetectorParameters::create();

//     0 = CORNER_REFINE_NONE,     ///< Tag and corners detection based on the ArUco approach
//     1 = CORNER_REFINE_SUBPIX,   ///< ArUco approach and refine the corners locations using corner subpixel accuracy
//     2 = CORNER_REFINE_CONTOUR,  ///< ArUco approach and refine the corners locations using the contour-points line fitting
//     3 = CORNER_REFINE_APRILTAG, ///< Tag and corners detection based on the AprilTag 2 approach @cite wang2016iros
      detectorParameters->cornerRefinementMethod = fm_context_.cv4_corner_refinement_method_;

      // Detect markers
      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f>> corners;
      cv::aruco::detectMarkers(gray_image, localization_aruco_dictionary_, corners, ids, detectorParameters);

      // return the corners as an observations structure.
      std::pair<std::vector<int>, std::vector<std::vector<cv::Point2f>>> args{ids, corners};
      return fvlam::Observations::from(args);
    }

    // Draw the boundary around detected markers.
    void annotate_image_with_detected_markers(cv::Mat &color_image,
                                              const Observations observations) override
    {
      // calculate colors
      const cv::Scalar borderColor(fm_context_.border_color_red_,
                                   fm_context_.border_color_green_,
                                   fm_context_.border_color_blue_);

      cv::Scalar textColor, cornerColor;
      textColor = cornerColor = borderColor;
      cv::swap(textColor.val[0], textColor.val[1]);     // text color just sawp G and R
      cv::swap(cornerColor.val[1], cornerColor.val[2]); // corner color just sawp G and B

      for (auto &observation : observations.observations()) {

        // draw marker sides
        for (int j = 0; j < 4; j++) {
          auto p0 = observation.corners_f_image()[j];
          auto p1 = observation.corners_f_image()[(j + 1) % 4];
          cv::line(color_image, p0.to<cv::Point2d>(), p1.to<cv::Point2d>(), borderColor, 1, cv::LINE_4);
        }

        // draw first corner mark
        cv::rectangle(color_image,
                      observation.corners_f_image()[0].to<cv::Point2d>() - cv::Point2d(3, 3),
                      observation.corners_f_image()[0].to<cv::Point2d>() + cv::Point2d(3, 3),
                      cornerColor, 1, cv::LINE_4);

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

    // Draw axes on a marker in an image.
    void annotate_image_with_marker_axis(cv::Mat &color_image,
                                         const Transform3 &t_camera_marker,
                                         const CameraInfo &camera_info) override
    {
      auto rvec = t_camera_marker.r().to<cv::Vec3d>();
      auto tvec = t_camera_marker.t().to<cv::Vec3d>();
      auto cc = camera_info.to<CvCameraCalibration>();

      cv::aruco::drawAxis(color_image,
                          cc.first, cc.second,
                          rvec, tvec, 0.1);
    }
  };


  template<>
  std::unique_ptr<FiducialMarkerInterface> make_fiducial_marker<FiducialMarkerCvContext>(
    const FiducialMarkerCvContext &fm_context, Logger &logger)
  {
    return std::make_unique<FiducialMarkerCv>(fm_context, logger);
  }
}
