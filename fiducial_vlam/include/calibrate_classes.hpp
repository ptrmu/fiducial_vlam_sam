
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
    bool valid_{false};
    double delta_scale_factor_{1.};
    std::array<cv::Point2f, 4> board_corners_f_image_{};

  public:
    BoardProjection() = default;

    explicit BoardProjection(std::vector<cv::Point2f> &board_corners, int max_image_dimension);

    inline const auto &valid() const
    { return valid_; }  //
    inline const auto &board_corners() const
    { return board_corners_f_image_; }  //

    double corner_pixel_delta(const BoardProjection &board_projection);
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
    ImageHolder(cv::Mat gray,
                const rclcpp::Time &time_stamp,
                cv::Ptr<cv::aruco::Dictionary> aruco_dictionary) :
      gray_{std::move(gray)}, time_stamp_{time_stamp},
      aruco_dictionary_{std::move(aruco_dictionary)}
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
    cv::Size image_size_;
    std::vector<std::shared_ptr<ImageHolder>> captured_images_{};

  public:
    explicit CapturedImages(cv::Size image_size) :
      image_size_{std::move(image_size)}
    {}

    // Make a shallow copy of the vector. This is how we pass images to another thread. The vector is not
    // thread safe and we have to make the other thread use a clone of the vector. The content of the vector
    // gets copied in this operation. But the content is a list of shared_ptrs. The items these shared pointers
    // point to are NOT copied. According to the stl docs, the control block of the shared_ptr is thread safe
    // and the object will be deleted correctly across multiple threads. The ImageHolders that are referenced
    // from this vector are not modified and therefor can be accessed by multiple threads.
    std::unique_ptr<const CapturedImages> shallow_clone() const;

    auto &image_size() const
    { return image_size_; } //
    auto &captured_images() const
    { return captured_images_; } //

    void capture(std::shared_ptr<ImageHolder> &image_holder);
  };
}
#endif //_CALIBRATE_CLASSES_HPP
