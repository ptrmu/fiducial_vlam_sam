#pragma once
#pragma ide diagnostic ignored "modernize-use-nodiscard"

#include "fvlam/logger.hpp"

namespace fiducial_vlam
{
  class LoggerRos2 : public fvlam::Logger
  {
    rclcpp::Node &node_;

    static int severity_from_level(Levels level)
    {
      switch (level) {
        default:
        case Levels::level_debug:
          return RCUTILS_LOG_SEVERITY_DEBUG;
        case Levels::level_info:
          return RCUTILS_LOG_SEVERITY_INFO;
        case Levels::level_warn:
          return RCUTILS_LOG_SEVERITY_WARN;
        case Levels::level_error:
          return RCUTILS_LOG_SEVERITY_ERROR;
        case Levels::level_fatal:
          return RCUTILS_LOG_SEVERITY_FATAL;
      }
    }

  public:
    explicit LoggerRos2(rclcpp::Node &node) :
      node_{node}
    {}

    // Test if the requested level is high enough for output
    bool test_level(Levels level) const override
    {
      return rcutils_logging_logger_is_enabled_for(node_.get_logger().get_name(), severity_from_level(level));
    }

    // Log a line of output at the specified level.
    void log_line(Levels level, std::string line) override
    {
      switch (level) {
        case Levels::level_debug:
          RCLCPP_DEBUG(node_.get_logger(), line.c_str());
          break;
        case Levels::level_info:
          RCLCPP_INFO(node_.get_logger(), line.c_str());
          break;
        case Levels::level_warn:
          RCLCPP_WARN(node_.get_logger(), line.c_str());
          break;
        case Levels::level_error:
          RCLCPP_ERROR(node_.get_logger(), line.c_str());
          break;
        case Levels::level_fatal:
          RCLCPP_FATAL(node_.get_logger(), line.c_str());
          break;
      }
    }
  };
}
