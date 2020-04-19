
#ifndef _CALIBRATE_CLASSES_HPP
#define _CALIBRATE_CLASSES_HPP

#include "opencv2/core.hpp"

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
                std::vector<int> aruco_ids,
                std::vector<std::vector<cv::Point2f> > aruco_corners,
                cv::Mat homo,
                BoardProjection board_projection) :
      gray_{gray},
      aruco_ids_{std::move(aruco_ids)}, aruco_corners_{std::move(aruco_corners)},
      homo_{std::move(homo)}, board_projection_{std::move(board_projection)}
    {}
  };
}
#endif //_CALIBRATE_CLASSES_HPP
