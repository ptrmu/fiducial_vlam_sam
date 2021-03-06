#ifndef FIDUCIAL_VLAM_OBSERVATION_HPP
#define FIDUCIAL_VLAM_OBSERVATION_HPP

#include <vector>

#include "fiducial_vlam_msgs/msg/observations.hpp"

namespace fiducial_vlam
{
// ==============================================================================
// Observation class
// ==============================================================================

  class Observation
  {
    // The id of the marker that we observed.
    int id_;

    double x0_, y0_;
    double x1_, y1_;
    double x2_, y2_;
    double x3_, y3_;

  public:
    Observation(int id,
                double x0, double y0,
                double x1, double y1,
                double x2, double y2,
                double x3, double y3)
      : id_(id),
        x0_(x0), y0_(y0),
        x1_(x1), y1_(y1),
        x2_(x2), y2_(y2),
        x3_(x3), y3_(y3)
    {}

    explicit Observation(int id)
      : id_(id),
        x0_(0.), y0_(0.),
        x1_(0.), y1_(0.),
        x2_(0.), y2_(0.),
        x3_(0.), y3_(0.)
    {}

    explicit Observation(const fiducial_vlam_msgs::msg::Observation &msg)
      : id_(msg.id),
        x0_(msg.x0), y0_(msg.y0),
        x1_(msg.x1), y1_(msg.y1),
        x2_(msg.x2), y2_(msg.y2),
        x3_(msg.x3), y3_(msg.y3)
    {}

    auto id() const
    { return id_; } //
    auto &x0() const
    { return x0_; } //
    auto &x1() const
    { return x1_; } //
    auto &x2() const
    { return x2_; } //
    auto &x3() const
    { return x3_; } //
    auto &y0() const
    { return y0_; } //
    auto &y1() const
    { return y1_; } //
    auto &y2() const
    { return y2_; } //
    auto &y3() const
    { return y3_; } //

    auto &x0()
    { return x0_; } //
    auto &x1()
    { return x1_; } //
    auto &x2()
    { return x2_; } //
    auto &x3()
    { return x3_; } //
    auto &y0()
    { return y0_; } //
    auto &y1()
    { return y1_; } //
    auto &y2()
    { return y2_; } //
    auto &y3()
    { return y3_; } //

    template<class TPoint>
    auto to_point_vector() const
    {
      return std::vector<TPoint>{
        TPoint{x0_, y0_},
        TPoint{x1_, y1_},
        TPoint{x2_, y2_},
        TPoint{x3_, y3_},
      };
    }

    template<class TPoint>
    void to_point_vector(std::vector<TPoint> &destination) const
    {
      destination.insert(destination.end(), {
        TPoint{x0_, y0_},
        TPoint{x1_, y1_},
        TPoint{x2_, y2_},
        TPoint{x3_, y3_}});
    }
  };

// ==============================================================================
// Observations class
// ==============================================================================

  class Observations
  {
    // The list of observations
    std::vector<Observation> observations_{};

  public:
    Observations() = default;

    explicit Observations(const fiducial_vlam_msgs::msg::Observations &msg);

    auto &observations() const
    { return observations_; } //
    auto &observations()
    { return observations_; } //
    auto size() const
    { return observations_.size(); } //

    void add(const Observation &observation); //

    fiducial_vlam_msgs::msg::Observations to_msg(std_msgs::msg::Header::_stamp_type stamp,
                                                 const std_msgs::msg::Header::_frame_id_type &frame_id,
                                                 const sensor_msgs::msg::CameraInfo &camera_info_msg);
  };


}
#endif //FIDUCIAL_VLAM_OBSERVATION_HPP
