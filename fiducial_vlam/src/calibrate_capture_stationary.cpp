
#include "calibrate.hpp"
#include "calibrate_classes.hpp"

namespace fiducial_vlam
{
// ==============================================================================
// CalibrateCaptureStationaryImpl class
// ==============================================================================

  class CalibrateCaptureStationaryImpl : public CalibrateCaptureInterface
  {
    class State : public CalibrateCaptureInterface
    {
      CalibrateCaptureStationaryImpl &impl_;

    protected:
      State(CalibrateCaptureStationaryImpl &impl) :
        impl_(impl)
      {}

      void _activate()
      {
        impl_.state_ = this;
      }
    };

    class Ready : public State
    {
    public:
      Ready(CalibrateCaptureStationaryImpl &impl) :
        State(impl)
      {}

      void process_image(const BoardProjection &board_projection,
                         cv_bridge::CvImage &color_marked) override
      {}

      void activate()
      {
        _activate();
      }
    };

    class Tracking : public State
    {
    public:
      Tracking(CalibrateCaptureStationaryImpl &impl) :
        State(impl)
      {}

      void process_image(const BoardProjection &board_projection,
                         cv_bridge::CvImage &color_marked) override
      {}

      void activate()
      {
        _activate();
      }
    };

    class Stationary : public State
    {
    public:
      Stationary(CalibrateCaptureStationaryImpl &impl) :
        State(impl)
      {}

      void process_image(const BoardProjection &board_projection,
                         cv_bridge::CvImage &color_marked) override
      {}

      void activate()
      {
        _activate();
      }
    };

    class Captured : public State
    {
    public:
      Captured(CalibrateCaptureStationaryImpl &impl) :
        State(impl)
      {}

      void process_image(const BoardProjection &board_projection,
                         cv_bridge::CvImage &color_marked) override
      {}

      void activate()
      {
        _activate();
      }
    };

    rclcpp::Logger &logger_;
    const CalibrateContext &cxt_;

    Ready ready_;
    Tracking tracking_;
    Stationary stationary_;
    Captured captured_;

    CalibrateCaptureInterface *state_;

  public:
    CalibrateCaptureStationaryImpl(rclcpp::Logger &logger,
                                   const CalibrateContext &cxt) :
      logger_(logger), cxt_(cxt),
      ready_(*this), tracking_(*this), stationary_(*this), captured_(*this),
      state_(nullptr)
    {
      ready_.activate();
    }

    void process_image(const BoardProjection &board_projection,
                       cv_bridge::CvImage &color_marked) override
    {}

  private:
  };


  std::unique_ptr<CalibrateCaptureInterface> make_calibrate_capture_stationary(rclcpp::Logger &logger,
                                                                               const CalibrateContext &cxt)
  {
    return std::make_unique<CalibrateCaptureStationaryImpl>(logger, cxt);
  }

}

