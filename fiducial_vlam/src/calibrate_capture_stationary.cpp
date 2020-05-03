
#include "calibrate.hpp"
#include "calibrate_classes.hpp"
#include "opencv2/imgproc.hpp"
#include "rclcpp/rclcpp.hpp"

namespace fiducial_vlam
{

  constexpr auto min_time_before_leave_ready = std::chrono::milliseconds(500);
  constexpr double min_time_stationary_secs = 4.0;
  constexpr double delta_threshold = 5.0;
  const cv::Scalar feedback_border_color_0(0, 0, 255);
  const cv::Scalar feedback_border_color_1(0, 255, 0);

// ==============================================================================
// draw_board_boundary
// ==============================================================================

  static void draw_board_boundary(cv::InputOutputArray image,
                                  const std::array<cv::Point2f, 4> &board_corners,
                                  double border_fraction_colored_1 = 0.,
                                  cv::Scalar border_color_0 = feedback_border_color_0,
                                  cv::Scalar border_color_1 = feedback_border_color_1)
  {
    border_fraction_colored_1 = std::min(1.0, std::max(0.0, border_fraction_colored_1));

    for (int j = 0; j < 4; j++) {
      double beg_fraction = j / 4.0;
      double end_fraction = beg_fraction + 0.25;
      auto p0 = board_corners[j];
      auto p1 = board_corners[(j + 1) % 4];
      auto pm = end_fraction <= border_fraction_colored_1 ? p1 :
                beg_fraction >= border_fraction_colored_1 ? p0 :
                (p0 * (1. - 4. * (border_fraction_colored_1 - beg_fraction)) +
                 p1 * (0. + 4. * (border_fraction_colored_1 - beg_fraction)));
      if (beg_fraction < border_fraction_colored_1) {
        cv::line(image, p0, pm, border_color_1, 3);
      }
      if (end_fraction > border_fraction_colored_1) {
        cv::line(image, pm, p1, border_color_0, 3);
      }
    }
  }

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
                        cv::Mat &color_marked) override
      {
        auto time_stamp{image_holder->time_stamp()};

        // We can only leave ready state when a board has been viewed for a small
        // amount of time.
        if (!image_holder->board_projection().valid()) {
          last_empty_time_ = time_stamp;
          return;
        }

        // Provide some feed back in this state.
        draw_board_boundary(color_marked, image_holder->board_projection().board_corners());

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
                        cv::Mat &color_marked) override
      {
        // If we are tracking and the board disappears, then
        // go back to ready mode
        if (!image_holder->board_projection().valid()) {
          impl_.ready_state_.activate(image_holder->time_stamp());
          return;
        }

        // Provide some feed back in this state.
        draw_board_boundary(color_marked, image_holder->board_projection().board_corners());

        // When the board becomes stationary, then transition to stationary mode.
        if (impl_.test_stationary(image_holder)) {
          impl_.stationary_state_.activate(image_holder);
          return;
        }

        // otherwise stay in the tracking state waiting for the
        // board to stop moving.
        impl_.reset_stationary(image_holder);
      }

      void activate(std::shared_ptr<ImageHolder> &image_holder)
      {
        _activate();
        impl_.reset_stationary(image_holder);
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
                        cv::Mat &color_marked) override
      {
        // If we are stationary and the board disappears, then
        // go back to ready mode
        if (!image_holder->board_projection().valid()) {
          impl_.ready_state_.activate(image_holder->time_stamp());
          return;
        }

        // Mark the color_marked image with a coloration that indicates how long
        // this board has been stationary.
        draw_board_boundary(color_marked, image_holder->board_projection().board_corners(),
                            (image_holder->time_stamp() - start_stationary_time_).seconds() / min_time_stationary_secs);

        // If the board starts moving then transition back to the
        // tracking state.
        if (!impl_.test_stationary(image_holder)) {
          impl_.tracking_state_.activate(image_holder);
          return;
        }

        // If the board stays stationary for the specified length of time
        // then transition to the captured state (and capture the image)
        if ((image_holder->time_stamp() - start_stationary_time_).seconds() > min_time_stationary_secs) {
          impl_.captured_state_.activate(image_holder);
          return;
        }

        // Stay in Stationary state
      }

      void activate(std::shared_ptr<ImageHolder> &image_holder)
      {
        _activate();
        start_stationary_time_ = image_holder->time_stamp();
      }
    };

    class CapturedState : public State
    {
      std::array<cv::Point2f, 4> captured_board_corners_;

    public:
      CapturedState(CalibrateCaptureStationaryImpl &impl) :
        State(impl)
      {}

      void test_capture(std::shared_ptr<ImageHolder> &image_holder,
                        cv::Mat &color_marked) override
      {
        // Stay in the captured state until the board is removed from
        // the view.
        if (!image_holder->board_projection().valid()) {
          impl_.ready_state_.activate(image_holder->time_stamp());
          return;
        }

        // Show an indication that the image has been captured.
        draw_board_boundary(color_marked, captured_board_corners_, 1.0);
      }

      void activate(std::shared_ptr<ImageHolder> &image_holder)
      {
        _activate();

        // Capture this image
        impl_.captured_images_.capture(image_holder);

        // record the board boundary so we can give stationary feedback that the image
        // has been captured.
        captured_board_corners_ = image_holder->board_projection().board_corners();
      }
    };

    rclcpp::Logger &logger_;
    const CalibrateContext &cxt_;
    CapturedImages &captured_images_;

    ReadyState ready_state_;
    TrackingState tracking_state_;
    StationaryState stationary_state_;
    CapturedState captured_state_;

    CalibrateCaptureInterface *state_;

    BoardProjection last_board_projection_{};

    void reset_stationary(std::shared_ptr<ImageHolder> &image_holder)
    {
      last_board_projection_ = image_holder->board_projection();
    }

    bool test_stationary(std::shared_ptr<ImageHolder> &image_holder)
    {
      return last_board_projection_.corner_pixel_delta(image_holder->board_projection()) < delta_threshold;
    }


  public:
    CalibrateCaptureStationaryImpl(rclcpp::Logger &logger,
                                   const CalibrateContext &cxt,
                                   const rclcpp::Time &now,
                                   CapturedImages &captured_images) :
      logger_(logger), cxt_(cxt), captured_images_{captured_images},
      ready_state_(*this), tracking_state_(*this), stationary_state_(*this), captured_state_(*this),
      state_(nullptr)
    {
      ready_state_.activate(now);
    }

    void test_capture(std::shared_ptr<ImageHolder> &image_holder,
                      cv::Mat &color_marked) override
    {
      state_->test_capture(image_holder, color_marked);
    }

  private:
  };


  std::unique_ptr<CalibrateCaptureInterface> make_calibrate_capture_stationary(rclcpp::Logger &logger,
                                                                               const CalibrateContext &cxt,
                                                                               const rclcpp::Time &now,
                                                                               CapturedImages &captured_images)
  {
    return std::make_unique<CalibrateCaptureStationaryImpl>(logger, cxt, now, captured_images);
  }

}

