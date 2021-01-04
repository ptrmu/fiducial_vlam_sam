#pragma once

#include "marker.hpp"
#include "observation.hpp"

namespace fvlam
{
// ==============================================================================
// ObservationsBundle class
// ==============================================================================

  class ObservationsBundle
  {
    MarkerMap map_;
    std::vector<Observations> obs_;

  };
}