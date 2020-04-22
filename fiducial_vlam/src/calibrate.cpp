
#include "calibrate.hpp"

#include "calibrate_classes.hpp"
#include "calibration_board_config.hpp"
#include "cv_bridge/cv_bridge.h"
#include "fiducial_math.hpp"
#include "observation.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "rclcpp/logging.hpp"
#include "ros2_shared/string_printf.hpp"
#include "task_thread.hpp"
#include "transform_with_covariance.hpp"

#include <map>

namespace fiducial_vlam
{

// ==============================================================================
// Drawing functions copied from opencv
// ==============================================================================

  static void drawDetectedMarkers(cv::InputOutputArray _image, cv::InputArrayOfArrays _corners,
                                  cv::InputArray _ids = cv::noArray(),
                                  cv::Scalar borderColor = cv::Scalar(0, 255, 0))
  {
    CV_Assert(_image.getMat().total() != 0 &&
              (_image.getMat().channels() == 1 || _image.getMat().channels() == 3));
    CV_Assert((_corners.total() == _ids.total()) || _ids.total() == 0);

    // calculate colors
    cv::Scalar textColor, cornerColor;
    textColor = cornerColor = borderColor;
    cv::swap(textColor.val[0], textColor.val[1]);     // text color just sawp G and R
    cv::swap(cornerColor.val[1], cornerColor.val[2]); // corner color just sawp G and B

    int nMarkers = (int) _corners.total();
    for (int i = 0; i < nMarkers; i++) {
      cv::Mat currentMarker = _corners.getMat(i);
      CV_Assert(currentMarker.total() == 4 && currentMarker.type() == CV_32FC2);

      // draw marker sides
      for (int j = 0; j < 4; j++) {
        cv::Point2f p0, p1;
        p0 = currentMarker.ptr<cv::Point2f>(0)[j];
        p1 = currentMarker.ptr<cv::Point2f>(0)[(j + 1) % 4];
        cv::line(_image, p0, p1, borderColor, 1);
      }
//      // draw first corner mark
//      cv::rectangle(_image, currentMarker.ptr<cv::Point2f>(0)[0] - cv::Point2f(3, 3),
//                    currentMarker.ptr<cv::Point2f>(0)[0] + cv::Point2f(3, 3), cornerColor, 1, cv::LINE_AA);

      // draw ID
//      if (_ids.total() != 0) {
//        cv::Point2f cent(0, 0);
//        for (int p = 0; p < 4; p++)
//          cent += currentMarker.ptr<cv::Point2f>(0)[p];
//        cent = cent / 4.;
//        std::stringstream s;
//        s << "id=" << _ids.getMat().ptr<int>(0)[i];
//        putText(_image, s.str(), cent, cv::FONT_HERSHEY_SIMPLEX, 0.5, textColor, 2);
//      }
    }
  }

  static void drawDetectedCornersCharuco(cv::InputOutputArray _image, cv::InputArray _charucoCorners,
                                         cv::InputArray _charucoIds = cv::noArray(),
                                         cv::Scalar cornerColor = cv::Scalar(255, 0, 0))
  {
    CV_Assert(_image.getMat().total() != 0 &&
              (_image.getMat().channels() == 1 || _image.getMat().channels() == 3));
    CV_Assert((_charucoCorners.getMat().total() == _charucoIds.getMat().total()) ||
              _charucoIds.getMat().total() == 0);

    unsigned int nCorners = (unsigned int) _charucoCorners.getMat().total();
    for (unsigned int i = 0; i < nCorners; i++) {
      cv::Point2f corner = _charucoCorners.getMat().at<cv::Point2f>(i);

      // draw first corner mark
      cv::rectangle(_image, corner - cv::Point2f(3, 3), corner + cv::Point2f(3, 3), cornerColor, 1, cv::LINE_AA);

      // draw ID
//      if (_charucoIds.total() != 0) {
//        int id = _charucoIds.getMat().at<int>(i);
//        std::stringstream s;
//        s << "id=" << id;
//        putText(_image, s.str(), corner + cv::Point2f(5, -5), cv::FONT_HERSHEY_SIMPLEX, 0.5,
//                cornerColor, 2);
//      }
    }
  }

  static void drawBoardCorners(cv::InputOutputArray image,
                               const std::vector<cv::Point2f> &board_corners,
                               cv::Scalar borderColor = cv::Scalar(0, 0, 255))
  {
    for (int j = 0; j < 4; j++) {
      auto p0 = board_corners[j];
      auto p1 = board_corners[(j + 1) % 4];
      cv::line(image, p0, p1, borderColor, 1);
    }
  }

  static void drawPolygonAtCenter(cv::Mat &color,
                                  std::vector<cv::Point2f> &board_corners,
                                  cv::Scalar borderColor = cv::Scalar(0, 0, 255))
  {
    cv::Point2f avg;
    for (int i = 0; i < board_corners.size(); i += 1) {
      avg = avg + board_corners[i];
    }
    avg = avg / double(board_corners.size());
    avg.x -= color.cols / 2;
    avg.y -= color.rows / 2;
    std::vector<cv::Point> bc = {
      cv::Point{int(round(board_corners[0].x - avg.x)), int(round(board_corners[0].y - avg.y))},
      cv::Point{int(round(board_corners[1].x - avg.x)), int(round(board_corners[1].y - avg.y))},
      cv::Point{int(round(board_corners[2].x - avg.x)), int(round(board_corners[2].y - avg.y))},
      cv::Point{int(round(board_corners[3].x - avg.x)), int(round(board_corners[3].y - avg.y))},
    };
    cv::fillConvexPoly(color, bc, borderColor);
//    for (int j = 0; j < 4; j++) {
//      cv::Point2f p0, p1;
//      p0 = board_corners[j];
//      p1 = board_corners[(j + 1) % 4];
//      line(image, p0, p1, borderColor, 1);
//    }
  }

#if 0

  struct CalibrationImage
  {
    BoardProjection target_;
    float difference_{};
    std::shared_ptr<ImageHolder> image_{};

    CalibrationImage(BoardProjection &&target) :
      target_{target}
    {}
  };

  class BoardTargets
  {
    cv::Size image_size_;
    std::vector<CalibrationImage> best_images_;

    std::vector<std::shared_ptr<ImageHolder>> captured_images_{};

    static BoardProjection new_target(double board_width_per_height, const cv::Size &image_size,
                                      int x_alignment, float x_normalized, float width_normalized,
                                      int y_alignment, float y_normalized)
    {
      double x_max = image_size.width - 1;
      double y_max = image_size.height - 1;

      double width = width_normalized * x_max;
      double height = width / board_width_per_height;

      double left = x_normalized * x_max;
      switch (x_alignment) {
        case 0:
          left -= width / 2;
          break;
        case 1:
          left -= width;
          break;
      }

      double top = y_normalized * y_max;
      switch (y_alignment) {
        case 0:
          top -= height / 2;
          break;
        case 1:
          top -= height;
          break;
      }

      return BoardProjection(std::vector<cv::Point2f>{
        cv::Point2f(left, top),
        cv::Point2f(left + width, top),
        cv::Point2f(left + width, top + height),
        cv::Point2f(left, top + height),
      });
    }

    static std::vector<CalibrationImage> new_best_images(float width_per_height, const cv::Size &image_size)
    {
      return std::vector<CalibrationImage>{
        CalibrationImage(new_target(width_per_height, image_size, -1, 0., 0.25, -1, 0.)),
        CalibrationImage(new_target(width_per_height, image_size, +0, .5, 0.25, -1, 0.)),
        CalibrationImage(new_target(width_per_height, image_size, +1, 1., 0.25, -1, 0.)),
        CalibrationImage(new_target(width_per_height, image_size, -1, 0., 0.25, +1, 1.)),
        CalibrationImage(new_target(width_per_height, image_size, +0, .5, 0.25, +1, 1.)),
        CalibrationImage(new_target(width_per_height, image_size, +1, 1., 0.25, +1, 1.)),
      };
    }

  public:
    explicit BoardTargets(rclcpp::Logger &logger, double board_width_per_height, const cv::Size &image_size) :
      image_size_{image_size}, best_images_{new_best_images(board_width_per_height, image_size)}
    {}

    void reset()
    {
      image_size_.width = 0;
    }

    std::vector<CalibrationImage> &get_best_images()
    {
      return best_images_;
    }

    int width()
    {
      return image_size_.width;
    }

    int height()
    {
      return image_size_.height;
    }

    std::vector<std::shared_ptr<ImageHolder>> &get_captured_images()
    {
      return captured_images_;
    }

    void capture_image(std::shared_ptr<ImageHolder> &image_holder)
    {
      captured_images_.emplace_back(image_holder);
    }

    void compare_to_targets(std::shared_ptr<ImageHolder> &image_holder)
    {
      for (auto &best_image : best_images_) {
        auto difference = best_image.target_.difference(image_holder->board_projection_);
        if (difference < 300.) {
          if (!best_image.image_ ||
              difference < best_image.difference_) {
            best_image.image_ = image_holder;
            best_image.difference_ = difference;
          }
        }
      }
    }
  };

  struct FrameData
  {
    std::vector<int> ids_;
    std::vector<std::vector<cv::Point2f> > corners_;
    cv::Mat currentCharucoCorners_;
    cv::Mat currentCharucoIds_;

    explicit FrameData(CharucoMath::CvCharucoMath &cvcm, cv::Mat &image)
    {
      std::vector<std::vector<cv::Point2f> > rejected;

      // detect markers
      cv::aruco::detectMarkers(image, cvcm.dictionary_, corners_, ids_, cvcm.detectorParams_, rejected);

      // refind strategy to detect more markers
      if (cvcm.cxt_.refind_strategy_) {
        cv::aruco::refineDetectedMarkers(image, cvcm.board_, corners_, ids_, rejected);
      }

      // interpolate charuco corners
      if (!ids_.empty())
        cv::aruco::interpolateCornersCharuco(corners_, ids_,
                                             image, cvcm.charucoboard_,
                                             currentCharucoCorners_, currentCharucoIds_);
    }
  };

  struct AllFramesData
  {
    // collect data from each frame
    std::vector<std::vector<std::vector<cv::Point2f>>> allCorners_;
    std::vector<std::vector<int>> allIds_;
    std::vector<cv::Mat> allImgs_;
    cv::Size imgSize_;

    explicit AllFramesData(CharucoMath::CvCharucoMath &cvcm,
                           const std::vector<std::shared_ptr<cv_bridge::CvImage>> &captured_images)
    {
      for (auto &color : captured_images) {
        FrameData fd{cvcm, color->image};
        allCorners_.emplace_back(fd.corners_);
        allIds_.emplace_back(fd.ids_);
        allImgs_.emplace_back(color->image);
        imgSize_ = color->image.size();
      }
    }
  };
#endif

// ==============================================================================
// CalibrateCameraProcessImageImpl class
// ==============================================================================

  class CalibrateCameraProcessImageImpl : public ProcessImageInterface
  {
    rclcpp::Logger &logger_;
    const CalibrateContext &cal_cxt_;
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    CharucoboardConfig cbm_;
    CapturedImages captured_images_;
    std::unique_ptr<CalibrateCaptureInterface> test_capture_;

    bool capture_next_image_{false};

  public:
    CalibrateCameraProcessImageImpl(rclcpp::Logger &logger,
                                    const CalibrateContext &cal_cxt,
                                    const rclcpp::Time &time_stamp,
                                    const cv::Size &image_size) :
      logger_{logger}, cal_cxt_{cal_cxt},
      detectorParams_{cv::aruco::DetectorParameters::create()},
      dictionary_{cv::aruco::getPredefinedDictionary(
        cv::aruco::PREDEFINED_DICTIONARY_NAME(cal_cxt.cal_aruco_dictionary_id_))},
      cbm_(cal_cxt.cal_squares_x_, cal_cxt.cal_squares_y_, cal_cxt.cal_square_length_,
           cal_cxt.cal_upper_left_white_not_black_, cal_cxt.cal_marker_length_),
      captured_images_(image_size),
      test_capture_(make_calibrate_capture_stationary(logger, cal_cxt, time_stamp, captured_images_))
    {
#if (CV_VERSION_MAJOR == 4)
//     0 = CORNER_REFINE_NONE,     ///< Tag and corners detection based on the ArUco approach
//     1 = CORNER_REFINE_SUBPIX,   ///< ArUco approach and refine the corners locations using corner subpixel accuracy
//     2 = CORNER_REFINE_CONTOUR,  ///< ArUco approach and refine the corners locations using the contour-points line fitting
//     3 = CORNER_REFINE_APRILTAG, ///< Tag and corners detection based on the AprilTag 2 approach @cite wang2016iros
      detectorParams_->cornerRefinementMethod = cal_cxt.cal_cv4_corner_refinement_method_;
#else
      // 0 = false
      // 1 = true
      detectorParameters->doCornerRefinement = cal_cxt.cal_cv3_do_corner_refinement_;
#endif
      RCLCPP_INFO(logger, "CalibrateCameraProcessImage created for %dx%d (wxh) images",
                  image_size.width, image_size.height);
    }

    static std::unique_ptr<CalibrateCameraProcessImageImpl> load_images(rclcpp::Logger &logger,
                                                                        const CalibrateContext &cal_cxt,
                                                                        const rclcpp::Time &now)
    {
      cv::FileStorage fs_header(std::string(cal_cxt.cal_images_file_name_).append(".yml"),
                                cv::FileStorage::READ);

      auto pi{std::make_unique<CalibrateCameraProcessImageImpl>(logger, cal_cxt, now,
                                                                cv::Size{static_cast<int>( fs_header["width"]),
                                                                         static_cast<int>(fs_header["height"])})};

      cv::FileNode file_names = fs_header["imageNames"];
      for (auto it = file_names.begin(); it != file_names.end(); ++it) {
        std::string image_name;
        (*it)["name"] >> image_name;

        cv::Mat gray{cv::imread(image_name, cv::IMREAD_ANYCOLOR)};

        auto image_holder = pi->new_image_holder(gray, now);
        pi->captured_images_.capture(image_holder);
      }

      return pi;
    }

    Observations process_image(std::shared_ptr<cv_bridge::CvImage> &gray,
                               const rclcpp::Time &time_stamp,
                               cv::Mat &color_marked) override
    {
      // Don't process images that happen to be a different size.
      if (gray->image.size[1] != captured_images_.image_size().width ||
          gray->image.size[0] != captured_images_.image_size().height) {
        return Observations{};
      }

      auto image_holder = new_image_holder(gray->image, time_stamp);

//      if (!image_holder->aruco_ids_.empty()) {
//        board_targets_->compare_to_targets(image_holder);
//      }

      // Check if a manual capture has been requested
      if (capture_next_image_) {
        captured_images_.capture(image_holder);
        capture_next_image_ = false;
      }

      // Evaluate if we should capture this image
      test_capture_->test_capture(image_holder, color_marked);

      if (color_marked.dims != 0) {

        // Annotate the image with info we have collected so far.
        if (!image_holder->aruco_ids_.empty()) {
          drawDetectedMarkers(color_marked, image_holder->aruco_corners_);
        }

//        if (!image_holder->board_projection_.ordered_board_corners_.empty()) {
//          drawBoardCorners(color_marked, image_holder->board_projection_.ordered_board_corners_);
//        }

        for (auto &captured_image : captured_images_()) {
          drawBoardCorners(color_marked,
                           captured_image->board_projection_.ordered_board_corners_,
                           cv::Scalar(255, 0, 0));
        }
      }

      // Detect the markers in this image and create a list of
      // observations.
      return Observations{};
    }

    TransformWithCovariance solve_t_map_camera(const Observations &observations,
                                               const CameraInfoInterface &camera_info,
                                               const Map &map) override
    {
      return TransformWithCovariance{};
    }

    std::string prep_image_capture()
    {
      capture_next_image_ = true;
      return std::string("An image will be captured.");
    }

    std::string load_images()
    {
      return std::string{};
    }

    std::string save_images()
    {
      cv::FileStorage fs_header(std::string(cal_cxt_.cal_images_file_name_).append(".yml"),
                                cv::FileStorage::WRITE);

      fs_header << "width" << captured_images_.image_size().width
                << "height" << captured_images_.image_size().height
                << "imageNames" << "[";

      auto captured_images = captured_images_();
      for (int i = 0; i < captured_images.size(); i += 1) {

        auto image_file_name{ros2_shared::string_print::f("%s_%03d.png", cal_cxt_.cal_images_file_name_.c_str(), i)};
        auto res = cv::imwrite(image_file_name, captured_images[i]->gray_);

        fs_header << "{:" << "name" << image_file_name << "},";
      }

      fs_header << "]";
      fs_header.release();
      return std::string{};
    }

    std::string status()
    {
      return ros2_shared::string_print::f("# captured images:%d, w:%d, h:%d",
                                          captured_images_().size(),
                                          captured_images_.image_size().width,
                                          captured_images_.image_size().height);
    }

    const std::vector<std::shared_ptr<ImageHolder>> &get_captured_images()
    {
      return captured_images_();
    }

  private:
    std::shared_ptr<ImageHolder> new_image_holder(const cv::Mat &gray, const rclcpp::Time &time_stamp)
    {
      std::vector<std::vector<cv::Point2f> > rejected;

      // detect markers
      std::vector<int> aruco_ids;
      std::vector<std::vector<cv::Point2f> > aruco_corners;
      cv::aruco::detectMarkers(gray, dictionary_, aruco_corners, aruco_ids, detectorParams_, rejected);

//      // refind strategy to detect more markers
//      if (cxt_.refind_strategy_) {
//        cv::aruco::refineDetectedMarkers(gray, board_, aruco_corners, aruco_ids, rejected);
//      }
//
//      // interpolate charuco corners
//      cv::Mat charuco_ids;
//      cv::Mat charuco_corners;
//      if (!aruco_ids.empty()) {
//        cv::aruco::interpolateCornersCharuco(aruco_corners, aruco_ids,
//                                             gray, charucoboard_,
//                                             charuco_corners, charuco_ids);
//      }
//
      // Calculate Homography
      cv::Mat homo;
      std::vector<cv::Point2f> board_corners;
      if (!aruco_ids.empty()) {

        std::vector<cv::Point2f> op{};
        std::vector<cv::Point2f> ip{};

        for (int i = 0; i < aruco_ids.size(); i += 1) {
          auto id = aruco_ids[i];
          auto object_points = cbm_.to_aruco_corners_f_facade(id);
          auto image_points = aruco_corners[i];
          for (int j = 0; j < 4; j += 1) {
            op.emplace_back(cv::Point2f{float(object_points(0, j)), float(object_points(1, j))});
            ip.emplace_back(cv::Point2f{float(image_points[j].x), float(image_points[j].y)});
          }
        }

        homo = cv::findHomography(op, ip);

        // Figure out the projection of the board corners in the image
        auto board_corners_f_board = cbm_.board_corners_f_facade_point2_array<cv::Point2f>();
        cv::perspectiveTransform(board_corners_f_board, board_corners, homo);
      }

      return std::make_shared<ImageHolder>(
        gray, time_stamp,
        std::move(aruco_ids), std::move(aruco_corners),
        std::move(homo), BoardProjection{std::move(board_corners)});
    }
  };

// ==============================================================================
// CalibrateCameraResult class
// ==============================================================================

  class CalibrateCameraResult
  {
  public:
    bool valid_{false};
    cv::Matx33d camera_matrix_{};
    cv::Matx<double, 5, 1> dist_coeffs_{};
  };

// ==============================================================================
// CalibrateCameraWork class
// ==============================================================================

  class CalibrateCameraWork
  {
    const std::vector<std::shared_ptr<ImageHolder>> captured_images_;
    CharucoboardConfig cbm_;

    using MarkersHomography = std::map<ArucoId, std::tuple<cv::Mat, std::size_t>>;

  public:
    CalibrateCameraWork(const CalibrateContext &cal_cxt,
                        const std::vector<std::shared_ptr<ImageHolder>> &captured_images) :
      captured_images_{captured_images},
      cbm_(cal_cxt.cal_squares_x_, cal_cxt.cal_squares_y_, cal_cxt.cal_square_length_,
           cal_cxt.cal_upper_left_white_not_black_, cal_cxt.cal_marker_length_)
    {}

    CalibrateCameraResult solve_calibration()
    {
      std::vector<std::vector<cv::Vec3f>> junctions_f_board;
      std::vector<std::vector<cv::Vec2f>> junctions_f_image;

      // Loop over the images finding the checkerboard junctions
      for (auto &captured_iamge : captured_images_) {
        interpolate_junction_locations(captured_iamge,
                                       junctions_f_board,
                                       junctions_f_image);
      }

      cv::Mat rvecs;
      cv::Mat tvecs;
      cv::Mat stdDeviationsIntrinsics;
      cv::Mat stdDeviationsExtrinsics;
      cv::Mat perViewErrors;

      CalibrateCameraResult res;

      auto err = calibrateCamera(junctions_f_board, junctions_f_image,
                                 cv::Size{captured_images_[0]->gray_.cols, captured_images_[0]->gray_.rows},
                                 res.camera_matrix_, res.dist_coeffs_,
                                 rvecs, tvecs,
                                 stdDeviationsIntrinsics,
                                 stdDeviationsExtrinsics,
                                 perViewErrors);

      res.valid_ = true;
      return res;
    }

  private:
    void interpolate_junction_locations(std::shared_ptr<ImageHolder> captured_image,
                                        std::vector<std::vector<cv::Vec3f>> &junctions_f_board,
                                        std::vector<std::vector<cv::Vec2f>> &junctions_f_image)
    {
      std::vector<cv::Vec3f> js_f_board{};
      std::vector<cv::Vec2f> js_f_image{};

      // Calculate the local homography for each found marker and build a map indexed by
      // the ArucoId.
      auto markers_homography = calculate_markers_homography(captured_image);

      // Walk over all the junctions on the board.
      for (JunctionId junction_id = 0; junction_id < cbm_.max_junction_id_; junction_id += 1) {

        // Get the two adjacent aruco ids.
        auto adjacent_aruco_ids = cbm_.get_adjacent_arucos(junction_id);
        auto adjacent_aruco_closest_corner_idx = cbm_.get_adjacent_arucos_closest_corner(junction_id);

        // Figure out where this junction is on the facade.
        auto junction_location = cbm_.junction_id_to_junction_location(junction_id);
        std::vector<cv::Point2f> junction_f_facade{cv::Point2f(junction_location(0), junction_location(1))};

        // For both of the adjacent aruco markers, check that they have been detected, and
        // use the local marker homography to figure out where the junction should be in the
        // image.
        std::vector<cv::Point2f> local_junctions_f_image{};
        std::vector<cv::Point2f> closest_corners_f_image{};
        for (std::size_t i = 0; i < adjacent_aruco_ids.size(); i += 1) {

          // Find the local homography for this marker
          auto find_ptr = markers_homography.find(adjacent_aruco_ids[i]);
          if (find_ptr != markers_homography.end()) {

            // Map the junction location on the board to a the junction location in the image using the
            // homography transformation of the adjacent aruco marker.
            std::vector<cv::Point2f> junction_f_image;
            cv::perspectiveTransform(junction_f_facade, junction_f_image, std::get<0>(find_ptr->second));
            local_junctions_f_image.emplace_back(junction_f_image[0]);

            // Pick out the location of the corner of this marker that is closest to the
            // junction.
            auto &aruco_corners = captured_image->aruco_corners_[std::get<1>(find_ptr->second)];
            closest_corners_f_image.emplace_back(aruco_corners[adjacent_aruco_closest_corner_idx[i]]);
          }
        }

        // If neither of the markers was found, then continue to the next
        // junction
        if (local_junctions_f_image.size() < 1) {
          continue;
        }

        // Average the junction image location if both of the markers have been detected.
        if (local_junctions_f_image.size() > 1) {
          local_junctions_f_image[0] = (local_junctions_f_image[0] + local_junctions_f_image[1]) / 2.0;
        }

        // We want to figure a custom window size for doing the sub-pixel corner refinement.
        // This is done by using a window size that is smaller than the distance from the
        // junction to the closest aruco corner.
        auto winSize = calculate_sub_pix_win_size(local_junctions_f_image[0], closest_corners_f_image);

        // Find the junction image location with sub pixel accuracy.
        local_junctions_f_image.resize(1);
        cv::cornerSubPix(captured_image->gray_, local_junctions_f_image, winSize, cv::Size(),
                         cv::TermCriteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS,
                                          30, DBL_EPSILON));

        // Add these junction locations (f_image, f_board) to the list
        js_f_board.emplace_back(cv::Vec3f(junction_location(0), junction_location(1), 0.));
        js_f_image.emplace_back(local_junctions_f_image[0]);
      }

      junctions_f_board.emplace_back(std::move(js_f_board));
      junctions_f_image.emplace_back(std::move(js_f_image));
    }

    MarkersHomography calculate_markers_homography(std::shared_ptr<ImageHolder> captured_image)
    {
      MarkersHomography markers_homography{};

      for (std::size_t idx = 0; idx < captured_image->aruco_ids_.size(); idx += 1) {
        ArucoId aruco_id(captured_image->aruco_ids_[idx]);
        auto &aruco_corners_f_image(captured_image->aruco_corners_[idx]);

        std::vector<cv::Point2f> aruco_corners_f_facade{};
        auto const &acff(cbm_.to_aruco_corners_f_facade(aruco_id));
        for (std::size_t c = 0; c < 4; c += 1) {
          aruco_corners_f_facade.emplace_back(cv::Point2f(acff(0, c), acff(1, c)));
        }

        auto homo = findHomography(aruco_corners_f_facade, aruco_corners_f_image);

        markers_homography.emplace(aruco_id, std::tuple<cv::Mat, std::size_t>{homo, idx});
      }

      return markers_homography;
    }

    // Figure out how big to make the window that will be used for the sub-pixel refinement to find
    // the image location at the junction of two black squares on the configuration target.
    // As input to the routine, we have the image coordinates where we think the junction will
    // be and the image coordinates of the closest corners of the aruco markers. We want the window
    // size as large as possible be it can't include the aruco corner because the sub-pixel algorithm
    // might lock on to the aruco corner instead of the black square junction.
    cv::Size calculate_sub_pix_win_size(cv::Point2f &mean_junction_f_image,
                                        std::vector<cv::Point2f> &closest_corners_f_image)
    {
      // Figure out the window using one of the aruco corners.
      auto size2f{cv::Size2f(std::abs(mean_junction_f_image.x - closest_corners_f_image[0].x),
                             std::abs(mean_junction_f_image.y - closest_corners_f_image[0].y))};

      // If two aruco markers were found, then pick the smallest window.
      if (closest_corners_f_image.size() > 1) {
        auto size2f1{cv::Size2f(std::abs(mean_junction_f_image.x - closest_corners_f_image[1].x),
                                std::abs(mean_junction_f_image.y - closest_corners_f_image[1].y))};
        size2f = cv::Size2f(std::min(size2f.width, size2f1.width),
                            std::min(size2f.height, size2f1.height));
      }

      cv::Size win_size{static_cast<int>(std::floor(size2f.width)),
                        static_cast<int>(std::floor(size2f.height))};

      // do some sanity checks.
      win_size = cv::Size{win_size.width - 2, win_size.height - 2}; // remove 2 pixels for safety
      win_size = cv::Size{std::min(10, std::max(1, win_size.width)),  // min: 1, max: 10
                          std::min(10, std::max(1, win_size.height))};
      return win_size;
    }
  };

// ==============================================================================
// CalibrateCameraTask class
// ==============================================================================

  class CalibrateCameraTask
  {
    task_thread::TaskThread<CalibrateCameraWork> task_thread_;
    std::future<CalibrateCameraResult> calibrate_camera_future_{};
    CalibrateCameraResult calibrate_camera_result_{};

  public:
    CalibrateCameraTask(const CalibrateContext &cal_cxt,
                        const std::vector<std::shared_ptr<ImageHolder>> &captured_images) :
      task_thread_{std::make_unique<CalibrateCameraWork>(cal_cxt, captured_images)}
    {}

    std::string check_completion()
    {
      // If the results are valid, then the useer has already been notified
      if (calibrate_camera_result_.valid_) {
        return std::string{};
      }

      // If the future is valid_, then the camera calibration task has been queued and we should check
      // to see if it is complete
      if (calibrate_camera_future_.valid()) {

        // Is it complete?
        bool complete = calibrate_camera_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready;
        if (!complete) {
          return std::string{};
        }

        calibrate_camera_result_ = calibrate_camera_future_.get();
        return std::string("Calibrate camera task complete.");
      }

      // The calibration task has not been queued, so queue it up.
      std::promise<CalibrateCameraResult> solve_map_promise{};
      calibrate_camera_future_ = solve_map_promise.get_future();

      auto func = [promise = std::move(solve_map_promise)](CalibrateCameraWork &ccw) mutable -> void
      {
        promise.set_value(ccw.solve_calibration());
      };

      task_thread_.push(std::move(func));

      return std::string("Calibrate camera task queued.");
    }

    bool calibration_complete()
    {
      return calibrate_camera_result_.valid_;
    }

    std::string save_calibration()
    {
      return std::string{""};
    }

    std::string status()
    {
      return ros2_shared::string_print::f("CalibrateCameraTask status: %s",
                                          calibrate_camera_result_.valid_ ? "done" :
                                          calibrate_camera_future_.valid() ? "working" : "pending");
    }
  };

// ==============================================================================
// CalibrateCameraImpl class
// ==============================================================================

  class CalibrateCameraImpl : public CalibrateCameraInterface
  {
    rclcpp::Logger &logger_;
    const CalibrateContext &cal_cxt_;
    std::unique_ptr<CalibrateCameraProcessImageImpl> pi_{};
    std::unique_ptr<CalibrateCameraTask> cct_{};

  public:
    explicit CalibrateCameraImpl(rclcpp::Logger &logger,
                                 const CalibrateContext &cal_cxt) :
      logger_{logger},
      cal_cxt_{cal_cxt}
    {}

    Observations process_image(std::shared_ptr<cv_bridge::CvImage> &gray,
                               const rclcpp::Time &time_stamp,
                               cv::Mat &color_marked) override
    {
      // The first time this is called, we have to initialize the targets with the size
      // of the image passed in.
      if (!pi_) {
        pi_ = std::make_unique<CalibrateCameraProcessImageImpl>(logger_, cal_cxt_, time_stamp,
                                                                cv::Size{gray->image.cols, gray->image.rows});
      }

      return pi_->process_image(gray, time_stamp, color_marked);
    }

    TransformWithCovariance solve_t_map_camera(const Observations &observations,
                                               const CameraInfoInterface &camera_info,
                                               const Map &map) override
    {
      return pi_ ? pi_->solve_t_map_camera(observations, camera_info, map) : TransformWithCovariance{};
    }

    std::string cal_cmd(const std::string &cmd,
                        const rclcpp::Time &now) override
    {
      std::string ret_str;

      if (cmd.compare("capture") == 0) {
        if (pi_) {
          ret_str = pi_->prep_image_capture();
        }

      } else if (cmd.compare("save_images") == 0) {
        if (pi_) {
          ret_str = pi_->save_images();
        }

      } else if (cmd.compare("status") == 0) {
        if (pi_) {
          ret_str = pi_->status();
          if (cct_) {
            ret_str.append("\n");
          }
        }
        if (cct_) {
          ret_str.append(cct_->status());
        }

      } else if (cmd.compare("load_images") == 0) {
        pi_.reset(nullptr);
        pi_ = CalibrateCameraProcessImageImpl::load_images(logger_, cal_cxt_, now);
        ret_str = pi_->status();

      } else if (cmd.compare("calibrate") == 0) {
        cct_.reset(nullptr);
        if (pi_ && pi_->get_captured_images().size() > 0) {
          cct_ = std::make_unique<CalibrateCameraTask>(cal_cxt_, pi_->get_captured_images());
          ret_str = std::string("Calibration queued.");
        } else {
          ret_str = std::string("Cannot calibrate with zero images.");
        }

      } else if (cmd.compare("save_calibration") == 0) {
        ret_str = do_save_calibration();

      } else if (cmd.compare("reset") == 0) {
        pi_.reset(nullptr);
        cct_.reset(nullptr);
      }

      return ret_str;
    }

    std::string on_timer() override
    {
      return cct_ ? cct_->check_completion() : std::string{};
    }

  private:

    std::string do_save_calibration()
    {
      if (!cct_) {
        return std::string("No calibration available");
      }

      if (!cct_->calibration_complete()) {
        return std::string("Calibration not complete");
      }

      return cct_->save_calibration();
    }
  };

  std::unique_ptr<CalibrateCameraInterface> make_calibrate_camera(rclcpp::Logger &logger,
                                                                  const CalibrateContext &cxt)
  {
    return std::make_unique<CalibrateCameraImpl>(logger, cxt);
  }

}
