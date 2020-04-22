
#ifndef _CALIBRATE_CLASSES_HPP
#define _CALIBRATE_CLASSES_HPP

#include "opencv2/core.hpp"
#include "rclcpp/rclcpp.hpp"

namespace fiducial_vlam
{

// ==============================================================================
// BoardProjection struct
// ==============================================================================

  struct BoardProjection
  {
    // Hold the image coordinates of the four corners of the board location.
    const std::vector<cv::Point2f> ordered_board_corners_;

    BoardProjection(std::vector<cv::Point2f> &&ordered_board_corners) :
      ordered_board_corners_{ordered_board_corners}
    {}

    float difference(const BoardProjection &other) const
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

// ==============================================================================
// ImageHolder struct
// ==============================================================================

  struct ImageHolder
  {
    const cv::Mat gray_;
    const rclcpp::Time time_stamp_;
    const std::vector<int> aruco_ids_;
    const std::vector<std::vector<cv::Point2f> > aruco_corners_;
    const cv::Mat homo_;
    const BoardProjection board_projection_;

    ImageHolder(const cv::Mat &gray,
                const rclcpp::Time &time_stamp,
                std::vector<int> aruco_ids,
                std::vector<std::vector<cv::Point2f> > aruco_corners,
                cv::Mat homo,
                BoardProjection board_projection) :
      gray_{gray}, time_stamp_{time_stamp},
      aruco_ids_{std::move(aruco_ids)}, aruco_corners_{std::move(aruco_corners)},
      homo_{std::move(homo)}, board_projection_{std::move(board_projection)}
    {}
  };

// ==============================================================================
// CapturedImages struct
// ==============================================================================

  class CapturedImages
  {
    const cv::Size image_size_;
    std::vector<std::shared_ptr<ImageHolder>> captured_images_{};

  public:
    explicit CapturedImages(const cv::Size &image_size) :
      image_size_{image_size}
    {}

    void capture(std::shared_ptr<ImageHolder> &image_holder)
    {
      captured_images_.emplace_back(image_holder);
    }

    const cv::Size image_size()
    {
      return image_size_;
    }

    const std::vector<std::shared_ptr<ImageHolder>> &operator()()
    {
      return captured_images_;
    }
  };
}
#endif //_CALIBRATE_CLASSES_HPP
