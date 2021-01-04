

#include "fvlam/camera_info.hpp"
#include "fvlam/logger.hpp"
#include "fvlam/marker.hpp"
#include "fvlam/observation.hpp"
#include "fvlam/observations_bundle.hpp"
#include "fvlam/transform3_with_covariance.hpp"
#include "opencv2/core/persistence.hpp"

namespace fvlam
{

// ==============================================================================
// MarkerMap save methods
// ==============================================================================

  template<>
  void fvlam::Translate3::to<cv::FileStorage>(cv::FileStorage &other) const
  {
    other << t_(0);
    other << t_(1);
    other << t_(2);
  }

  template<>
  void fvlam::Rotate3::to<cv::FileStorage>(cv::FileStorage &other) const
  {
    auto r = xyz();
    other << r(2);
    other << r(1);
    other << r(0);
  }

  template<>
  void fvlam::Marker::to<cv::FileStorage>(cv::FileStorage &other) const
  {
    other << "{";

    other << "id" << int(id_);
    other << "f" << (is_fixed_ ? 1 : 0);
    other << "xyz" << "[";
    t_world_marker_.tf().t().to(other);
    other << "]";
    other << "rpy" << "[";
    t_world_marker_.tf().r().to(other);
    other << "]";

    other << "}";
  }

  template<>
  void fvlam::MarkerMap::to<cv::FileStorage>(cv::FileStorage &other) const
  {
    other << "marker_length" << marker_length();
    other << "markers" << "[";

    for (auto &marker : markers_) {
      marker.second.to(other);
    }

    other << "]";
  }


// ==============================================================================
// MarkerMap load methods
// ==============================================================================

  struct FileStorageContext
  {
    fvlam::Logger &logger_;
    cv::FileNode node_;
    bool &success_;

    FileStorageContext(fvlam::Logger &logger, cv::FileNode node, bool &success) :
      logger_{logger}, node_{node}, success_{success}
    {}

    cv::FileNode operator()()
    {
      return node_;
    }

    FileStorageContext make(cv::FileNode file_node)
    {
      return FileStorageContext{logger_, file_node, success_};
    }

    void set_failed()
    {
      success_ = false;
    }

    bool success()
    {
      return success_;
    }
  };

  template<>
  Translate3 fvlam::Translate3::from<FileStorageContext>(FileStorageContext &other)
  {
    double x = other()[0];
    double y = other()[1];
    double z = other()[2];
    return Translate3{x, y, z};
  }

  template<>
  Rotate3 fvlam::Rotate3::from<FileStorageContext>(FileStorageContext &other)
  {
    double rx = other()[0];
    double ry = other()[1];
    double rz = other()[2];
    return Rotate3::RzRyRx(rx, ry, rz);
  }

  template<>
  Marker fvlam::Marker::from<FileStorageContext>(FileStorageContext &other)
  {
    int id = other()["id"];
    int fixed = other()["f"];

    auto t_node = other()["xyz"];
    auto t_context = other.make(t_node);
    auto t = Translate3::from(t_context);

    auto r_node = other()["rpy"];
    auto r_context = other.make(r_node);
    auto r = Rotate3::from(r_context);

    return fvlam::Marker{std::uint64_t(id), Transform3WithCovariance{Transform3{r, t}}, fixed != 0};
  }

  template<>
  MarkerMap fvlam::MarkerMap::from<FileStorageContext>(FileStorageContext &other)
  {
    double marker_length = other()["marker_length"];
    MarkerMap map{marker_length};

    auto markers_node = other()["markers"];
    for (auto it = markers_node.begin(); it != markers_node.end(); ++it) {
      auto marker_context = other.make(*it);
      auto marker = Marker::from(marker_context);
      map.add_marker(marker);
    }

    return map;
  }

// ==============================================================================
// MarkerMap class
// ==============================================================================

  void MarkerMap::save(const std::string filename, Logger &logger) const
  {
    cv::FileStorage fs(filename, cv::FileStorage::WRITE | cv::FileStorage::FORMAT_YAML);
    if (!fs.isOpened()) {
      logger.error() << "Could not create MarkerMap file :" << filename;
      return;
    }

    to(fs);
  }

  fvlam::MarkerMap MarkerMap::load(const std::string filename, Logger &logger)
  {
    cv::FileStorage fs(filename, cv::FileStorage::READ | cv::FileStorage::FORMAT_YAML);
    if (!fs.isOpened()) {
      logger.error() << "Could not open MarkerMap file :" << filename;
      return MarkerMap{0.0};
    }

    bool success{true};
    FileStorageContext marker_map_context{logger, fs.root(), success};

    auto map = MarkerMap::from(marker_map_context);
    return success ? map : MarkerMap{0.0};
  }

}