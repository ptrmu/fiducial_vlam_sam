#pragma once
#pragma ide diagnostic ignored "modernize-use-nodiscard"
#pragma ide diagnostic ignored "NotImplementedFunctions"

#include "camera_info.hpp"
#include "marker.hpp"
#include "observation.hpp"

namespace fvlam
{
// ==============================================================================
// ObservationsBundle class
// ==============================================================================

  class ObservationsBundle
  {
    CameraInfo camera_info_;
    Observations observations_;

  public:
    ObservationsBundle(CameraInfo camera_info, Observations observations) :
      camera_info_{std::move(camera_info)}, observations_{std::move(observations)}
    {}

    template<class T>
    static ObservationsBundle from(T &other);

    template<class T>
    T to() const;

    template<class T>
    void to(T &other) const;

    std::string to_string(bool also_cov = false) const;

    bool equals(const ObservationsBundle &other, double tol = 1.0e-9, bool check_relative_also = true) const;
  };

  class ObservationsBundles
  {
    MarkerMap map_;
    std::vector<ObservationsBundle> bundles_;

  public:
    explicit ObservationsBundles(MarkerMap map) :
      map_{std::move(map)}
    {}

    void add_bundle(const ObservationsBundle &bundle)
    {
      bundles_.emplace_back(bundle);
    }

    template<class T>
    static ObservationsBundles from(T &other);

    template<class T>
    T to() const;

    template<class T>
    void to(T &other) const;

    std::string to_string(bool also_cov = false) const;

    bool equals(const ObservationsBundles &other, double tol = 1.0e-9, bool check_relative_also = true) const;

    void save(const std::string &filename, Logger &logger) const; //
    static ObservationsBundles load(const std::string &filename, Logger &logger); //
  };
}
