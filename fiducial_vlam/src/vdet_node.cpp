
#include "cv_bridge/cv_bridge.h"
#include "fiducial_vlam/fiducial_vlam.hpp"
#include "fiducial_vlam_msgs/msg/observations_stamped.hpp"
#include "fvlam/camera_info.hpp"
#include "fvlam/localize_camera_interface.hpp"
#include "fvlam/logger.hpp"
#include "fvlam/marker.hpp"
#include "fvlam/observation.hpp"
#include <gtsam/base/timing.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "logger_ros2.hpp"
#include "observation_maker.hpp"
#include "vdet_context.hpp"

namespace fiducial_vlam
{

  struct VdetDiagnostics
  {
    std::uint64_t sub_camera_info_count_{0};
    std::uint64_t sub_image_raw_count_{0};
    std::uint64_t empty_observations_count_{0};
    std::uint64_t pub_observations_count_{0};
    std::uint64_t pub_image_marked_count_{0};
    rclcpp::Time start_time_;

    explicit VdetDiagnostics(const rclcpp::Time &start_time) :
      start_time_{start_time}
    {}

    void report(fvlam::Logger &logger, const rclcpp::Time &end_time);
  };

// ==============================================================================
// VdetNode class
// ==============================================================================

  class VdetNode : public rclcpp::Node
  {
    LoggerRos2 logger_;
    VdetDiagnostics diagnostics_;

    VdetContext cxt_{};

    std::unique_ptr<ObservationMakerInterface> observation_maker_{};

    void validate_parameters()
    {}

    void setup_parameters()
    {
#undef PAMA_PARAM
#define PAMA_PARAM(n, t, d) PAMA_PARAM_INIT(n, t, d)
      PAMA_PARAMS_INIT((*this), cxt_, , VDET_ALL_PARAMS, validate_parameters)

#undef PAMA_PARAM
#define PAMA_PARAM(n, t, d) PAMA_PARAM_CHANGED(n, t, d)
      PAMA_PARAMS_CHANGED((*this), cxt_, , VDET_ALL_PARAMS, validate_parameters, RCLCPP_INFO)

#undef PAMA_PARAM
#define PAMA_PARAM(n, t, d) PAMA_PARAM_LOG(n, t, d)
      PAMA_PARAMS_LOG((*this), cxt_, , VDET_ALL_PARAMS, RCLCPP_INFO)

#undef PAMA_PARAM
#define PAMA_PARAM(n, t, d) PAMA_PARAM_CHECK_CMDLINE(n, t, d)
      PAMA_PARAMS_CHECK_CMDLINE((*this), , VDET_ALL_PARAMS, RCLCPP_ERROR)
    }

  public:
    explicit VdetNode(const rclcpp::NodeOptions &options) :
      Node{"vdet_node", options},
      logger_{*this},
      diagnostics_{now()}
    {
      // Get parameters from the command line
      setup_parameters();

      // The observation_maker will capture the image and camera_info,
      // detect the markers in the image, and then publish the observations.
      observation_maker_ = make_observation_maker(
        cxt_, *this, logger_,
        [this](const fvlam::CameraInfoMap &,
               const fvlam::ObservationsSynced &) -> void
        {});

      RCLCPP_INFO(get_logger(), "vdet_node ready");
    }


  private:
  };

  std::shared_ptr<rclcpp::Node> vdet_node_factory(const rclcpp::NodeOptions &options)
  {
    return std::shared_ptr<rclcpp::Node>(new VdetNode(options));
  }
}
