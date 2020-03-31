
#include "calibrate.hpp"

#include "cv_bridge/cv_bridge.h"
#include "fiducial_math.hpp"
#include "observation.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/aruco/charuco.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "rclcpp/logging.hpp"
#include "transform_with_covariance.hpp"

namespace fiducial_vlam
{

// ==============================================================================
// BoardProjection struct
// ==============================================================================

  struct BoardProjection
  {
    // Hold the image coordinates of the four corners of the board location.
    std::vector<cv::Point2f> ordered_board_corners_;

    BoardProjection(std::vector<cv::Point2f> &&ordered_board_corners) :
      ordered_board_corners_{ordered_board_corners}
    {}

    float difference(BoardProjection &other)
    {
      auto diff = (
                    cv::norm(ordered_board_corners_[0] - other.ordered_board_corners_[0]) +
                    cv::norm(ordered_board_corners_[1] - other.ordered_board_corners_[1]) +
                    cv::norm(ordered_board_corners_[2] - other.ordered_board_corners_[2]) +
                    cv::norm(ordered_board_corners_[3] - other.ordered_board_corners_[3])
                  ) / 4;
      return diff;
    }
  };

  struct ImageHolder
  {
    cv::Mat gray_;

    std::vector<int> aruco_ids_;
    std::vector<std::vector<cv::Point2f> > aruco_corners_;

    cv::Mat homo_;
    BoardProjection board_projection_;

    ImageHolder(cv::Mat &gray,
                std::vector<int> &&aruco_ids, std::vector<std::vector<cv::Point2f> > &&aruco_corners,
                cv::Mat &&homo, BoardProjection &&board_projection) :
      gray_{gray},
      aruco_ids_{aruco_ids}, aruco_corners_{aruco_corners},
      homo_{homo}, board_projection_{board_projection}
    {}
  };

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
    cv::Size image_size_{0, 0};
    std::vector<CalibrationImage> best_images_{};

    static BoardProjection new_target(float width_per_height, const cv::Size &image_size,
                                      int x_alignment, float x_normalized, float width_normalized,
                                      int y_alignment, float y_normalized)
    {
      float x_max = image_size.width - 1;
      float y_max = image_size.height - 1;

      float width = width_normalized * x_max;
      float height = width / width_per_height;

      float left = x_normalized * x_max;
      switch (x_alignment) {
        case 0:
          left -= width / 2;
          break;
        case 1:
          left -= width;
          break;
      }

      float top = y_normalized * y_max;
      switch (y_alignment) {
        case 0:
          top -= height / 2;
          break;
        case 1:
          top -= height;
          break;
      }

      return BoardProjection(std::vector<cv::Point2f>{
        cv::Point2f{left, top},
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
//    explicit BoardTargets(rclcpp::Logger &logger, float width_per_height, const cv::Size &image_size) :
//      logger_{logger}, image_size{image_size}, best_images_{new_best_images(width_per_height, image_size)}
//    {
//    }

    bool check_init(const cv::MatSize &mat_size)
    {
      // Pick up this image size if we are not initialized
      if (image_size_.width == 0) {
        image_size_ = cv::Size(mat_size[1], mat_size[0]);
        best_images_.clear();
        // create new_best images
        return true;
      }

      // Don't process images that happen to be a different size.
      return image_size_.width == mat_size[1] && image_size_.height == mat_size[0];
    }

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

#if 0
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
// CalibrateCameraImpl class
// ==============================================================================

  class CalibrateCameraImpl : public CalibrateCameraInterface
  {
    rclcpp::Logger &logger_;
    const CalibrateContext &cal_cxt_;

    BoardTargets board_targets{};

  public:
    explicit CalibrateCameraImpl(rclcpp::Logger &logger,
                                 const CalibrateContext &cal_cxt) :
      logger_{logger},
      cal_cxt_{cal_cxt}
    {}

    Observations process_image(std::shared_ptr<cv_bridge::CvImage> &gray,
                               cv_bridge::CvImage &color_marked) override
    {
      // The first time this is called, we have to initialize the targets with the size
      // of the image passed in.
      if (!board_targets.check_init(gray->image.size)) {
        return Observations{};
      }

      auto image_holder = new_image_holder(gray);
//
//      if (!image_holder->aruco_ids_.empty()) {
//        board_targets_->compare_to_targets(image_holder);
//      }
//
//      // Annotate the image with info we have collected so far.
//      if (!image_holder->aruco_ids_.empty()) {
//        drawDetectedMarkers(marked, image_holder->aruco_corners_);
//      }
//
//      if (!image_holder->board_projection_.ordered_board_corners_.empty()) {
//        drawBoardCorners(marked, image_holder->board_projection_.ordered_board_corners_);
//      }
//
//      mark_best_images(marked);

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

    std::string calibrate_camera_cmd(const std::string cmd) override
    {
      return std::string{};
    }

  private:


    std::shared_ptr<ImageHolder> new_image_holder(cv::Mat &gray)
    {
//      std::vector<std::vector<cv::Point2f> > rejected;
//
//      // detect markers
//      std::vector<int> aruco_ids;
//      std::vector<std::vector<cv::Point2f> > aruco_corners;
//      cv::aruco::detectMarkers(gray, dictionary_, aruco_corners, aruco_ids, detectorParams_, rejected);
//
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
//      // Calculate Homography
//      cv::Mat homo;
//      std::vector<cv::Point2f> board_corners;
//      if (!aruco_ids.empty()) {
//        CharucoBoardModel cbm{cxt_.squares_x_, cxt_.squares_y_,
//                              cxt_.square_length_, cxt_.marker_length_};
//
//        std::vector<cv::Vec2f> op{};
//        std::vector<cv::Vec2f> ip{};
//
//        for (int i = 0; i < aruco_ids.size(); i += 1) {
//          auto id = aruco_ids[i];
//          auto object_points = cbm.marker_corners2D_f_board(id);
//          auto image_points = aruco_corners[i];
//          for (int j = 0; j < 4; j += 1) {
//            op.emplace_back(cv::Vec2f{float(object_points[j].x), float(object_points[j].y)});
//            ip.emplace_back(cv::Vec2f{float(image_points[j].x), float(image_points[j].y)});
//          }
//        }
//
//        homo = cv::findHomography(op, ip);
//
//        // Figure out the projection of the board corners in the image
//        auto board_corners_f_board = cbm.board_corners2D_f_board();
//        cv::perspectiveTransform(board_corners_f_board, board_corners, homo);
//      }
//
//      return std::make_shared<ImageHolder>(
//        gray,
//        std::move(aruco_ids), std::move(aruco_corners),
//        std::move(homo), BoardProjection{std::move(board_corners)});
    }

  };

  std::unique_ptr<CalibrateCameraInterface> make_calibrate_camera(rclcpp::Logger &logger,
                                                                  const CalibrateContext &cxt)
  {
    return std::make_unique<CalibrateCameraImpl>(logger, cxt);
  }

}
