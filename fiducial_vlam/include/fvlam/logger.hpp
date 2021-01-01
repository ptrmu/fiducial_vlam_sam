#pragma once

#include <iostream>
#include <memory>
#include <sstream>

// ==============================================================================
// Logger class
// ==============================================================================

namespace fvlam
{

  class Logger
  {
  public:
    enum Levels
    {
      level_debug = 0,
      level_info,
      level_warn,
      level_error,
      level_fatal,
    };

    class StreamProxy
    {
      Logger &logger_;
      Levels level_;
      bool test_level_;
      std::ostringstream oss_;

    public:
      StreamProxy(Logger &logger, Levels level) :
        logger_{logger},
        level_{level},
        test_level_{logger.test_level(level)},
        oss_{}
      {}

      ~StreamProxy()
      {
        logger_.log_line(level_, oss_.str());
      }

      template<typename V>
      StreamProxy &operator<<(V const &value)
      {
        if (test_level_) {
          oss_ << value;
        }
        return *this;
      }

      StreamProxy &operator<<(std::basic_ostream<char> &(*func)(std::basic_ostream<char> &))
      {
        if (test_level_) {
          func(oss_);
        }
        return *this;
      }
    };

  private:
    // Test if the requested level is high enough for output
    virtual bool test_level(Levels level) const = 0;

    // Log a line of output at the specified level.
    virtual void log_line(Levels level, std::string line) = 0;

  public:
    Logger() = default;

    virtual ~Logger() = default;

    Logger(Logger const &) = delete; //
    void operator=(Logger const &x) = delete; //

    // These methods return a new StreamProxy object that will stream or not.
    // This temporary StreamProxy object is held alive while the subsequent << operators
    // pass a reference to it along. The StreamProxy object is then destroyed at the end
    // of the statement. This is convenient compiler magic.
    StreamProxy debug()
    { return StreamProxy{*this, level_debug}; } //
    StreamProxy info()
    { return StreamProxy{*this, level_info}; } //
    StreamProxy warn()
    { return StreamProxy{*this, level_warn}; } //
    StreamProxy error()
    { return StreamProxy{*this, level_error}; } //
    StreamProxy fatal()
    { return StreamProxy{*this, level_fatal}; } //


    auto output_debug() const
    { return test_level(level_debug); } //
    auto output_info() const
    { return test_level(level_info); } //
    auto output_warn() const
    { return test_level(level_warn); } //
    auto output_error() const
    { return test_level(level_error); } //
    auto output_fatal() const
    { return test_level(level_fatal); } //
  };

// ==============================================================================
// LoggerCallbackCout class
// ==============================================================================

  class LoggerCout : public Logger
  {
    Logger::Levels output_level_;

  public:
    explicit LoggerCout(Logger::Levels output_level) :
      output_level_{output_level}
    {}

    // Test if the requested level is high enough for output
    bool test_level(Levels level) const override
    { return level >= output_level_; }

    // Log a line of output at the specified level.
    void log_line(Levels level, std::string line) override
    {
      if (test_level(level)) {
        std::cout << line << std::endl;
      }
    }
  };
}
