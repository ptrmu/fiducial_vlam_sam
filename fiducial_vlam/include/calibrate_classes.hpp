
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
}
#endif //_CALIBRATE_CLASSES_HPP
