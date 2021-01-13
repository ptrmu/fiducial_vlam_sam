#pragma once
#pragma ide diagnostic ignored "modernize-use-nodiscard"
#pragma ide diagnostic ignored "NotImplementedFunctions"
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"
#pragma ide diagnostic ignored "OCUnusedStructInspection"
#pragma ide diagnostic ignored "OCUnusedTypeAliasInspection"

#include <memory>

#include "transform3_with_covariance.hpp"

namespace fvlam
{
  class Logger; //
  class CameraInfo; //
  class MarkerMap; //
  class Observation; //
  class Observations; //

// ==============================================================================
// BuildMarkerMapInterface class
// ==============================================================================

// An interface used to localize a camera from fiducial marker observations.
  class LocalizeCameraInterface
  {
  public:
    virtual ~LocalizeCameraInterface() = default;

    // given observations of fiducial markers and a map of world locations of those
    // markers, figure out the camera pose in the world frame.
    virtual Transform3WithCovariance solve_t_map_camera(const Observations &observations,
                                                        const CameraInfo &camera_info,
                                                        const MarkerMap &map) = 0;

    // Given the observations that have been added so far, create and return a marker_map.
    virtual Transform3WithCovariance solve_t_camera_marker(const Observation &observation,
                                                           const CameraInfo &camera_info) = 0;
  };

  template<class TLcContext>
  std::unique_ptr<LocalizeCameraInterface> make_localize_camera(const TLcContext &lc_context,
                                                                Logger &logger,
                                                                const MarkerMap &map_initial);

// ==============================================================================
// FiducialMarkerInterface class
// ==============================================================================

// An interface used to deal with fiducial markers.
  class FiducialMarkerInterface
  {
  public:
    virtual ~FiducialMarkerInterface() = default;

    // given observations of fiducial markers and a map of world locations of those
    // markers, figure out the camera pose in the world frame.
    virtual Transform3WithCovariance solve_t_map_camera(const Observations &observations,
                                                        const CameraInfo &camera_info,
                                                        const MarkerMap &map) = 0;

    // Given the observations that have been added so far, create and return a marker_map.
    virtual Transform3WithCovariance solve_t_camera_marker(const Observation &observation,
                                                           const CameraInfo &camera_info) = 0;
  };

  template<class TLcContext>
  std::unique_ptr<FiducialMarkerInterface> make_localize_camera(const TLcContext &lc_context,
                                                                Logger &logger,
                                                                const MarkerMap &map_initial);

}
