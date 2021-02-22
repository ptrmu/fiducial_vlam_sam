#pragma once
#pragma ide diagnostic ignored "modernize-use-nodiscard"
#pragma ide diagnostic ignored "NotImplementedFunctions"

#include "camera_info.hpp"
#include "marker.hpp"
#include "observation.hpp"

namespace fvlam
{

// ==============================================================================
// ObservationsSeries class
// ==============================================================================

  class ObservationsSeries
  {
    MarkerMap map_;
    CameraInfoMap camera_info_map_;
    std::vector<ObservationsSynced> v_;

  public:
    ObservationsSeries(MarkerMap map, CameraInfoMap camera_info_map) :
      map_{std::move(map)}, camera_info_map_{std::move(camera_info_map)}, v_{}
    {}

    ObservationsSeries(MarkerMap map, CameraInfoMap camera_info_map,
                       std::vector<ObservationsSynced> v) :
      map_{std::move(map)}, camera_info_map_{std::move(camera_info_map)}, v_{std::move(v)}
    {}

    auto &map() const
    { return map_; }

    auto &camera_info_map() const
    { return camera_info_map_; }

    auto &v_mutable()
    { return v_; }

    auto &v() const
    { return v_; }

    template<class T>
    static ObservationsSeries from(T &other);

    template<class T>
    T to() const;

    template<class T>
    void to(T &other) const;

    std::string to_string() const;

    bool equals(const ObservationsSeries &other, double tol = 1.0e-9, bool check_relative_also = true) const;

    void save(const std::string &filename, Logger &logger) const; //
    static ObservationsSeries load(const std::string &filename, Logger &logger); //
  };
}