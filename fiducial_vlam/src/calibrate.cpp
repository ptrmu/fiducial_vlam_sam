
#include "calibrate.hpp"

#include "calibrate_classes.hpp"
#include "calibration_board_config.hpp"
#include "cv_bridge/cv_bridge.h"
#include "cv_utils.hpp"
#include "fiducial_math.hpp"
#include "observation.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "rclcpp/logging.hpp"
#include "ros2_shared/string_printf.hpp"
#include "task_thread.hpp"
#include "transform_with_covariance.hpp"

namespace fiducial_vlam
{

  constexpr auto time_display_captured_image_marked = std::chrono::milliseconds(1500);

// ==============================================================================
// BoardProjection class
// ==============================================================================

  static double calc_delta_scale_factor(std::vector<cv::Point2f> &board_corners,
                                        int max_image_dimension)
  {
    double longest_side{0.};
    for (int i = 0; i < 4; i += 1) {
      auto side = cv::norm(board_corners[i] - board_corners[(i + 1) % 4]);
      longest_side = std::max(side, longest_side);
    }
    return max_image_dimension / 4. / longest_side;
  }

  BoardProjection::BoardProjection(std::vector<cv::Point2f> &board_corners,
                                   int max_image_dimension) :
    valid_{true},
    delta_scale_factor_{calc_delta_scale_factor(board_corners, max_image_dimension)},
    board_corners_f_image_{board_corners[0], board_corners[1],
                           board_corners[2], board_corners[3]}
  {}

  double BoardProjection::corner_pixel_delta(const BoardProjection &board_projection)
  {
    auto &other_board_corners_f_image = board_projection.board_corners();

    // Calculate the number of pixels that each corner moved from this
    // location.
    double delta{0.};
    for (int i = 0; i < 4; i += 1) {
      delta += cv::norm(other_board_corners_f_image[i] - board_corners_f_image_[i]);
    }

    // A heuristic metric that seems to work OK for figuring out when
    // the board is not moving. We may need some normalization based on
    // the frame rate - but maybe not.
    delta *= delta_scale_factor_;

    return delta;
  }

// ==============================================================================
// ImageHolder class
// ==============================================================================

  std::shared_ptr<ImageHolder> ImageHolder::make(const cv::Mat &gray,
                                                 const rclcpp::Time &time_stamp,
                                                 const cv::Ptr<cv::aruco::Dictionary> &aruco_dictionary,
                                                 const CharucoboardConfig &cbm)
  {
    auto image_holder{std::make_shared<ImageHolder>(gray, time_stamp, aruco_dictionary)};
    image_holder->detect_markers(cbm, false);
    return image_holder;
  }

  void ImageHolder::detect_markers(const CharucoboardConfig &cbm,
                                   bool precise_not_quick)
  {
    auto detectorParams{cv::aruco::DetectorParameters::create()};

#if (CV_VERSION_MAJOR == 4)
    //     0 = CORNER_REFINE_NONE,     ///< Tag and corners detection based on the ArUco approach
    //     1 = CORNER_REFINE_SUBPIX,   ///< ArUco approach and refine the corners locations using corner subpixel accuracy
    //     2 = CORNER_REFINE_CONTOUR,  ///< ArUco approach and refine the corners locations using the contour-points line fitting
    //     3 = CORNER_REFINE_APRILTAG, ///< Tag and corners detection based on the AprilTag 2 approach @cite wang2016iros
    detectorParams->cornerRefinementMethod = precise_not_quick ?
                                             cv::aruco::CORNER_REFINE_CONTOUR :
                                             cv::aruco::CORNER_REFINE_NONE;
#else
    // 0 = false
    // 1 = true
    detectorParameters->doCornerRefinement = precise_not_quick;
#endif

    // detect markers
    cv::aruco::detectMarkers(gray_,
                             aruco_dictionary_,
                             aruco_corners_,
                             aruco_ids_,
                             detectorParams);

    // Calculate Homography and board corners
    if (!aruco_ids_.empty()) {

      std::vector<cv::Point2f> op{};
      std::vector<cv::Point2f> ip{};

      for (int i = 0; i < aruco_ids_.size(); i += 1) {
        auto id = aruco_ids_[i];
        auto object_points = cbm.to_aruco_corners_f_facade(id);
        auto image_points = aruco_corners_[i];
        for (int j = 0; j < 4; j += 1) {
          op.emplace_back(cv::Point2f{float(object_points(0, j)), float(object_points(1, j))});
          ip.emplace_back(cv::Point2f{float(image_points[j].x), float(image_points[j].y)});
        }
      }

      auto homo = cv::findHomography(op, ip);

      // Figure out the projection of the board corners in the image
      std::vector<cv::Point2f> board_corners;
      auto board_corners_f_board = cbm.board_corners_f_facade_point2_array<cv::Point2f>();
      cv::perspectiveTransform(board_corners_f_board, board_corners, homo);

      board_projection_ = BoardProjection{board_corners, std::max(gray_.cols, gray_.rows)};
    }
  }


  void CapturedImages::capture(std::shared_ptr<ImageHolder> &image_holder)
  {
    captured_images_.emplace_back(image_holder);
  }

// ==============================================================================
// CapturedImages struct
// ==============================================================================

  std::unique_ptr<const CapturedImages> CapturedImages::shallow_clone() const
  {
    auto ci = std::make_unique<CapturedImages>(image_size_);
    ci->captured_images_.insert(ci->captured_images_.end(), captured_images_.begin(), captured_images_.end());
    return ci;
  }

// ==============================================================================
// CalibrateCameraProcessImageImpl class
// ==============================================================================

  class CalibrateCameraProcessImageImpl : public ProcessImageInterface
  {
    rclcpp::Logger &logger_;
    const CalibrateContext &cal_cxt_;
    cv::Ptr<cv::aruco::Dictionary> aruco_dictionary_;
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
      aruco_dictionary_{cv::aruco::getPredefinedDictionary(
        cv::aruco::PREDEFINED_DICTIONARY_NAME(cal_cxt.cal_aruco_dictionary_id_))},
      cbm_(cal_cxt.cal_squares_x_, cal_cxt.cal_squares_y_, cal_cxt.cal_square_length_,
           cal_cxt.cal_upper_left_white_not_black_, cal_cxt.cal_marker_length_),
      captured_images_(image_size),
      test_capture_(make_calibrate_capture_stationary(logger, cal_cxt, time_stamp, captured_images_))
    {
      RCLCPP_INFO(logger, "CalibrateCameraProcessImage created for %dx%d (wxh) images",
                  image_size.width, image_size.height);
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

      auto image_holder = make_image_holder(gray->image, time_stamp);

      // Check if a manual capture has been requested
      if (capture_next_image_) {
        captured_images_.capture(image_holder);
        capture_next_image_ = false;
      }

      // Evaluate if we should capture this image
      test_capture_->test_capture(image_holder, color_marked);

      if (color_marked.dims != 0) {

        // Annotate the image with info we have collected so far.
        if (!image_holder->aruco_ids().empty()) {
          AnnotateImages::with_detected_markers(color_marked,
                                                image_holder->aruco_corners(),
                                                image_holder->aruco_ids());
        }

        for (auto &captured_image : captured_images_.captured_images()) {
          AnnotateImages::with_board_corners(color_marked,
                                             captured_image->board_projection().board_corners(),
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

      auto captured_images = captured_images_.captured_images();
      for (int i = 0; i < captured_images.size(); i += 1) {

        auto image_file_name{ros2_shared::string_print::f("%s_%03d.png", cal_cxt_.cal_images_file_name_.c_str(), i)};
        auto res = cv::imwrite(image_file_name, captured_images[i]->gray());

        fs_header << "{:"
                  << "name" << image_file_name
                  << "stamp" << std::to_string(captured_images[i]->time_stamp().nanoseconds())
                  << "clock" << captured_images[i]->time_stamp().get_clock_type()
                  << "},";
      }

      fs_header << "]";
      fs_header.release();
      return std::string{};
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
        std::string time_str;
        rcl_clock_type_t clock;
        (*it)["name"] >> image_name;
        (*it)["stamp"] >> time_str;
        (*it)["clock"] >> clock;

        cv::Mat gray{cv::imread(image_name, cv::IMREAD_ANYCOLOR)};

        auto image_holder = pi->make_image_holder(gray, rclcpp::Time(std::stoul(time_str), clock));
        pi->captured_images_.capture(image_holder);
      }

      return pi;
    }

    std::string status()
    {
      return ros2_shared::string_print::f("# captured images:%d, w:%d, h:%d",
                                          captured_images_.captured_images().size(),
                                          captured_images_.image_size().width,
                                          captured_images_.image_size().height);
    }

    const auto &captured_images()
    {
      return captured_images_;
    }

  private:
    std::shared_ptr<ImageHolder> make_image_holder(const cv::Mat &gray, const rclcpp::Time &time_stamp)
    {
      return ImageHolder::make(gray, time_stamp, aruco_dictionary_, cbm_);
    }
  };

// ==============================================================================
// CalibrateCameraThread class
// ==============================================================================

  class CalibrateCameraThread
  {
    task_thread::TaskThread<CalibrateCameraTaskInterface> task_thread_;
    std::future<CalibrateCameraTaskResult> calibrate_camera_future_{};
    CalibrateCameraTaskResult calibrate_camera_task_result_{};
    bool calibration_complete_{false};

  public:
    CalibrateCameraThread(const CalibrateContext &cal_cxt,
                          const rclcpp::Time &now,
                          std::unique_ptr<const CapturedImages> captured_images) :
      task_thread_{make_calibrate_camera_task(cal_cxt, now, std::move(captured_images)),
                   !cal_cxt.cal_compute_on_thread_}
    {}

    std::string check_completion(const rclcpp::Time &now)
    {
      // If the results are valid, then the user has already been notified
      if (calibration_complete_) {
        return std::string{};
      }

      // If the future is valid_, then the camera calibration task has been queued and we should check
      // to see if it is complete
      if (calibrate_camera_future_.valid()) {

        // Is it complete?
        calibration_complete_ =
          calibrate_camera_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready;
        if (!calibration_complete_) {
          return std::string{};
        }

        // Get the results from the task
        calibrate_camera_task_result_ = calibrate_camera_future_.get();
        return calibrate_camera_task_result_.calibration_report_;
      }

      // The calibration task has not been queued, so queue it up.
      std::promise<CalibrateCameraTaskResult> solve_map_promise{};
      calibrate_camera_future_ = solve_map_promise.get_future();

      auto func = [promise = std::move(solve_map_promise)](CalibrateCameraTaskInterface &ccw) mutable -> void
      {
        promise.set_value(ccw.calculate_calibration());
      };

      task_thread_.push(std::move(func));

      return std::string("Calibrate camera task queued.");
    }

    bool calibration_complete()
    {
      return calibration_complete_;
    }

    std::string status()
    {
      return ros2_shared::string_print::f("CalibrateCameraThread status: %s",
                                          calibration_complete_ ? "done" :
                                          calibrate_camera_future_.valid() ? "working" : "pending");
    }

    std::vector<cv::Mat> &get_captured_images_marked()
    {
      return calibrate_camera_task_result_.captured_images_marked_;
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
    std::unique_ptr<CalibrateCameraThread> cct_{};
    std::size_t marked_index_{0};
    rclcpp::Time last_time_display_marked_{0, 0, RCL_ROS_TIME};

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
        cct_.reset(nullptr);
        pi_ = CalibrateCameraProcessImageImpl::load_images(logger_, cal_cxt_, now);
        ret_str = pi_->status();

      } else if (cmd.compare("calibrate") == 0) {
        cct_.reset(nullptr);
        if (pi_ && !pi_->captured_images().captured_images().empty()) {
          cct_ = std::make_unique<CalibrateCameraThread>(cal_cxt_, now, pi_->captured_images().shallow_clone());
          ret_str = std::string("Calibration queued.");
        } else {
          ret_str = std::string("Cannot calibrate with zero images.");
        }

      } else if (cmd.compare("reset") == 0) {
        if (cct_) {
          cct_.reset(nullptr);
        } else {
          pi_.reset(nullptr);
        }
      }

      return ret_str;
    }

    std::string on_timer(const rclcpp::Time &now) override
    {
      return cct_ ? cct_->check_completion(now) : std::string{};
    }

    bool calibration_complete() override
    {
      return cct_ && cct_->calibration_complete();
    }

    void get_captured_image_marked(const rclcpp::Time &now,
                                   cv::Mat &captured_image_marked) override
    {
      // Can not return a marked captured image unless the calibration is complete
      if (!cct_ || !cct_->calibration_complete()) {
        return;
      }

      auto &cim = cct_->get_captured_images_marked();
      if (cim.size() < 0) {
        return;
      }

      // Switch images every now and then
      if ((now - last_time_display_marked_) > time_display_captured_image_marked) {
        marked_index_ += 1;
        last_time_display_marked_ = now;
      }

      marked_index_ = marked_index_ % cim.size();
      captured_image_marked = cim[marked_index_];
    }
  };

  std::unique_ptr<CalibrateCameraInterface> make_calibrate_camera(rclcpp::Logger &logger,
                                                                  const CalibrateContext &cxt)
  {
    return std::make_unique<CalibrateCameraImpl>(logger, cxt);
  }

}
