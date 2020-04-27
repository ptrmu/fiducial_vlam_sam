
#ifndef _CALIBRATE_CLASSES_HPP
#define _CALIBRATE_CLASSES_HPP

#include "opencv2/core.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cv
{
  namespace aruco
  {
    class Dictionary;
  }
}

namespace fiducial_vlam
{
  class CharucoboardConfig;

// ==============================================================================
// BoardProjection class
// ==============================================================================

  class BoardProjection
  {
    std::vector<cv::Point2f> ordered_board_corners_;

  public:
    BoardProjection() :
      ordered_board_corners_{}
    {}

    BoardProjection(std::vector<cv::Point2f> &ordered_board_corners) :
      ordered_board_corners_{ordered_board_corners}
    {}

    inline const auto &ordered_board_corners() const
    { return ordered_board_corners_; }  //

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
// ImageHolder class
// ==============================================================================

  class ImageHolder
  {
    cv::Mat gray_;
    rclcpp::Time time_stamp_;
    cv::Ptr<cv::aruco::Dictionary> aruco_dictionary_;

    std::vector<int> aruco_ids_{};
    std::vector<std::vector<cv::Point2f>> aruco_corners_{};
    BoardProjection board_projection_{};

  public:
    ImageHolder(const cv::Mat &gray,
                const rclcpp::Time &time_stamp,
                const cv::Ptr<cv::aruco::Dictionary> &aruco_dictionary) :
      gray_{gray}, time_stamp_{time_stamp},
      aruco_dictionary_{aruco_dictionary}
    {}

    static std::shared_ptr<ImageHolder> make(const cv::Mat &gray,
                                             const rclcpp::Time &time_stamp,
                                             const cv::Ptr<cv::aruco::Dictionary> &aruco_dictionary,
                                             const CharucoboardConfig &cbm);

    inline const auto &gray() const
    { return gray_; }  //
    inline const auto &time_stamp() const
    { return time_stamp_; } //
    inline const auto &aruco_ids() const
    { return aruco_ids_; } //
    inline const auto &aruco_corners() const
    { return aruco_corners_; } //
    inline const auto &board_projection() const
    { return board_projection_; } //

    void detect_markers(const CharucoboardConfig &cbm,
                        bool precise_not_quick);
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
