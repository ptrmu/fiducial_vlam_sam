
#include "calibrate.hpp"
#include "calibrate_classes.hpp"
#include "rclcpp/rclcpp.hpp"

namespace fiducial_vlam
{

  constexpr auto min_time_before_leave_ready = std::chrono::milliseconds(500);
  constexpr auto min_time_stationary = std::chrono::milliseconds(2000);
  constexpr double delta_threshold = 1.0;

// ==============================================================================
// drawBoardCorners
// ==============================================================================

//  static void drawBoardCorners(cv::InputOutputArray image,
//                               const std::vector<cv::Point2f> &board_corners,
//                               cv::Scalar borderColor = cv::Scalar(0, 0, 255))
//  {
//    for (int j = 0; j < 4; j++) {
//      auto p0 = board_corners[j];
//      auto p1 = board_corners[(j + 1) % 4];
//      cv::line(image, p0, p1, borderColor, 1);
//    }
//  }

// ==============================================================================
// StationaryBoard class
// ==============================================================================

  class StationaryBoard
  {
    std::vector<cv::Point2f> last_board_corners_{};

  public:
    void reset(std::shared_ptr<ImageHolder> &image_holder)
    {
      assert(image_holder->board_projection_.ordered_board_corners_.size() == 4);
      last_board_corners_ = image_holder->board_projection_.ordered_board_corners_;
    }

    bool test_stationary(std::shared_ptr<ImageHolder> &image_holder)
    {
      auto &board_corners = image_holder->board_projection_.ordered_board_corners_;
      assert(board_corners.size() == 4);

      // Calculate the number of pixels that each corner moved since the
      // last test. Calculate the average change in position of the four
      // corners and then divide by the time to get the pixel velocity
      // of the corners.
      double delta{0.};
      double longest_side{0.};
      for (int i = 0; i < 4; i += 1) {
        delta += cv::norm(board_corners[i] - last_board_corners_[i]);
        last_board_corners_[i] = board_corners[i];
        auto side = cv::norm(board_corners[i] - board_corners[(i + 1) % 4]);
        longest_side = std::max(side, longest_side);
      }

      delta /= 4. * longest_side * 0.001;

//      std::cout << delta << std::endl;

      return delta < delta_threshold;
    }

    std::vector<cv::Point2f> &last_board_corners() {
      return last_board_corners_;
    }
  };

// ==============================================================================
// CalibrateCaptureStationaryImpl class
// ==============================================================================

  class CalibrateCaptureStationaryImpl : public CalibrateCaptureInterface
  {

    class State : public CalibrateCaptureInterface
    {
    protected:
      CalibrateCaptureStationaryImpl &impl_;

      State(CalibrateCaptureStationaryImpl &impl) :
        impl_(impl)
      {}

      void _activate()
      {
        impl_.state_ = this;
      }
    };

    class ReadyState : public State
    {
      rclcpp::Time last_empty_time_;

    public:
      ReadyState(CalibrateCaptureStationaryImpl &impl) :
        State(impl)
      {}

      void test_capture(std::shared_ptr<ImageHolder> &image_holder,
                        cv_bridge::CvImage &color_marked) override
      {
        auto time_stamp{image_holder->time_stamp_};

        // We can only leave ready state when a board has been viewed for a small
        // amount of time.
        if (image_holder->board_projection_.ordered_board_corners_.empty()) {
          last_empty_time_ = time_stamp;
          return;
        }

        // Enforce the minimum time.
        if (time_stamp - last_empty_time_ < min_time_before_leave_ready) {
          return;
        }

        // Transition to tracking state.
        impl_.tracking_state_.activate(image_holder);
      }

      void activate(const rclcpp::Time now)
      {
        _activate();
        last_empty_time_ = now;
      }
    };

    class TrackingState : public State
    {
    public:
      TrackingState(CalibrateCaptureStationaryImpl &impl) :
        State(impl)
      {}

      void test_capture(std::shared_ptr<ImageHolder> &image_holder,
                        cv_bridge::CvImage &color_marked) override
      {
        // If we are tracking and the board disappears, then
        // go back to ready mode
        if (image_holder->board_projection_.ordered_board_corners_.empty()) {
          impl_.ready_state_.activate(image_holder->time_stamp_);
          return;
        }

        // When the board becomes stationary, then transition to stationary mode.
        if (impl_.stationary_board_.test_stationary(image_holder)) {
          impl_.stationary_state_.activate(image_holder);
          return;
        }

        // otherwise stay in the tracking state waiting for the
        // board to stop moving.
      }

      void activate(std::shared_ptr<ImageHolder> &image_holder)
      {
        _activate();
        impl_.stationary_board_.reset(image_holder);
      }
    };

    class StationaryState : public State
    {
      rclcpp::Time start_stationary_time_;

    public:
      StationaryState(CalibrateCaptureStationaryImpl &impl) :
        State(impl)
      {}

      void test_capture(std::shared_ptr<ImageHolder> &image_holder,
                        cv_bridge::CvImage &color_marked) override
      {
        // If we are stationary and the board disappears, then
        // go back to ready mode
        if (image_holder->board_projection_.ordered_board_corners_.empty()) {
          impl_.ready_state_.activate(image_holder->time_stamp_);
          return;
        }

        // If the board starts moving then transition back to the
        // tracking state.
        if (!impl_.stationary_board_.test_stationary(image_holder)) {
          impl_.tracking_state_.activate(image_holder);
          return;
        }

        // Mark the color_marked image with a coloration that indicates how long
        // this board has been stationary.

        // If the board stays stationary for the specified length of time
        // then transition to the captured state (and capture the image)
        if (image_holder->time_stamp_ - start_stationary_time_ > min_time_stationary) {
          impl_.captured_state_.activate(image_holder);
          return;
        }

        // Stay in Stationary state
      }

      void activate(std::shared_ptr<ImageHolder> &image_holder)
      {
        _activate();
        start_stationary_time_ = image_holder->time_stamp_;
      }
    };

    class CapturedState : public State
    {
    public:
      CapturedState(CalibrateCaptureStationaryImpl &impl) :
        State(impl)
      {}

      void test_capture(std::shared_ptr<ImageHolder> &image_holder,
                        cv_bridge::CvImage &color_marked) override
      {
        // Stay in the captured state until the board is removed from
        // the view.
        if (image_holder->board_projection_.ordered_board_corners_.empty()) {
          impl_.ready_state_.activate(image_holder->time_stamp_);
          return;
        }

        // Show an indication that the image has been captured.
      }

      void activate(std::shared_ptr<ImageHolder> &image_holder)
      {
        _activate();
      }
    };

    rclcpp::Logger &logger_;
    const CalibrateContext &cxt_;

    ReadyState ready_state_;
    TrackingState tracking_state_;
    StationaryState stationary_state_;
    CapturedState captured_state_;

    CalibrateCaptureInterface *state_;

    StationaryBoard stationary_board_{};

  public:
    CalibrateCaptureStationaryImpl(rclcpp::Logger &logger,
                                   const CalibrateContext &cxt,
                                   const rclcpp::Time &now) :
      logger_(logger), cxt_(cxt),
      ready_state_(*this), tracking_state_(*this), stationary_state_(*this), captured_state_(*this),
      state_(nullptr)
    {
      ready_state_.activate(now);
    }

    void test_capture(std::shared_ptr<ImageHolder> &image_holder,
                      cv_bridge::CvImage &color_marked) override
    {
      state_->test_capture(image_holder, color_marked);
    }

  private:
  };


  std::unique_ptr<CalibrateCaptureInterface> make_calibrate_capture_stationary(rclcpp::Logger &logger,
                                                                               const CalibrateContext &cxt,
                                                                               const rclcpp::Time &now)
  {
    return std::make_unique<CalibrateCaptureStationaryImpl>(logger, cxt, now);
  }

}

