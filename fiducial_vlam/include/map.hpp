#ifndef FIDUCIAL_VLAM_MAP_H
#define FIDUCIAL_VLAM_MAP_H

#include <map>

#include "convert_util.hpp"
#include "transform_with_covariance.hpp"

#include "fiducial_vlam_msgs/msg/map.hpp"

// coordinate frame conventions
//  t_destination_source is a transformation from source frame to destination frame
//  xxx_f_destination means xxx is expressed in destination frame

namespace cv_bridge
{
  class CvImage;
}

namespace fiducial_vlam
{
  class Observations;

// ==============================================================================
// Marker class
// ==============================================================================

  class Marker
  {
    // The id of the marker
    int id_{};

    // The pose of the marker in the map frame
    TransformWithCovariance t_map_marker_;

    // If not empty then 3D locations of corners.
    std::vector<Vector3WithCovariance> corners_f_map_;

    // Prevent modification if true
    bool is_fixed_{false};

    // Count of updates
    int update_count_{};

  public:
    Marker() = default;

    Marker(int id, TransformWithCovariance t_map_marker) :
      id_(id), t_map_marker_(std::move(t_map_marker)), update_count_(1), corners_f_map_{}
    {}

    Marker(int id, TransformWithCovariance t_map_marker, std::vector<Vector3WithCovariance> &corners_f_map) :
      id_(id), t_map_marker_(std::move(t_map_marker)), update_count_(1), corners_f_map_{corners_f_map}
    {}

    auto id() const
    { return id_; }

    auto is_fixed() const
    { return is_fixed_; }

    void set_is_fixed(bool is_fixed)
    { is_fixed_ = is_fixed; }

    auto update_count() const
    { return update_count_; }

    void set_update_count(int update_count)
    { update_count_ = update_count; }

    const auto &t_map_marker() const
    { return t_map_marker_; }

    void set_t_map_marker(TransformWithCovariance t_map_marker)
    { t_map_marker_ = std::move(t_map_marker); }

    const auto &corners_f_map() const
    { return corners_f_map_; }

    void set_corners_f_map(const std::vector<Vector3WithCovariance> &corners_f_map)
    { corners_f_map_ = corners_f_map; }

    bool has_corners()
    { return corners_f_map_.size() == 4; }
  };

// ==============================================================================
// Map class
// ==============================================================================

  class Map
  {
  public:
    enum MapStyles
    {
      pose = 0,
      covariance,
      corners
    };

  private:
    const enum MapStyles map_style_;
    const double marker_length_;
    std::map<int, Marker> markers_{};

  public:
    Map() = delete;

    explicit Map(MapStyles map_style, double marker_length_);

    explicit Map(const fiducial_vlam_msgs::msg::Map &msg);

    void reset(const Map &map); // Figure out how to remove this function

    const auto &markers() const
    { return markers_; }

    auto marker_length() const
    { return marker_length_; }

    auto map_style() const
    { return map_style_; }

    Marker *find_marker(int id);

    const Marker *find_marker_const(int id) const;

    void add_marker(Marker marker);

    std::unique_ptr<fiducial_vlam_msgs::msg::Map>
    to_map_msg(const std_msgs::msg::Header &header_msg);

    std::vector<TransformWithCovariance> find_t_map_markers(const Observations &observations);
  };

// ==============================================================================
// Utility
// ==============================================================================

//  void log_tf_transform(rclcpp::Node &node, std::string s, const tf2::Transform &transform);

} // namespace fiducial_vlam

#endif //FIDUCIAL_VLAM_MAP_H
