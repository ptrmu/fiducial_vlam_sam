#pragma ide diagnostic ignored "modernize-use-nodiscard"

#include "fvlam/camera_info.hpp"
#include "fvlam/logger.hpp"
#include "fvlam/marker.hpp"
#include "fvlam/observation.hpp"
#include "fvlam/observations_series.hpp"
#include "fvlam/transform3_with_covariance.hpp"
#include "opencv2/core/persistence.hpp"

namespace fvlam
{

// ==============================================================================
// FileStorageContext class
// ==============================================================================

  struct FileStorageContext
  {
    Logger &logger_;
    bool success_{true};

    class Node
    {
      FileStorageContext &cxt_;
      cv::FileNode node_;

    public:
      Node(FileStorageContext &cxt, const cv::FileNode &node)
        : cxt_{cxt}, node_{node}
      {
        (void) cxt;
      }

      Node make(const cv::FileNode &file_node)
      {
        return Node{cxt_, file_node};
      }

      cv::FileNode operator()()
      { return node_; }

//      FileStorageContext &cxt()
//      { return cxt_; }
    };

    explicit FileStorageContext(Logger &logger) :
      logger_{logger}
    {
      (void) logger_;
    }

    Node make(const cv::FileNode &file_node)
    {
      return Node{*this, file_node};
    }

    void set_failed()
    {
      success_ = false;
    }

    bool success() const
    {
      return success_;
    }
  };

// ==============================================================================
// Matrix from/to methods
// ==============================================================================

  template<class T1D>
  static T1D from_1d(FileStorageContext::Node &other)
  {
    T1D t1d;
    for (size_t r = 0; r < T1D::MaxSizeAtCompileTime; r += 1) {
      t1d(r) = other()[r];
    }
    return t1d;
  }

  template<class T1D>
  static void to_1d(const T1D &v, cv::FileStorage &other)
  {
    other << "[";
    for (size_t r = 0; r < T1D::MaxSizeAtCompileTime; r += 1) {
      other << v(r);
    }
    other << "]";
  }

  template<class T2D>
  static T2D from_2d(FileStorageContext::Node &other)
  {
    T2D t2d;
    for (std::size_t r = 0; r < T2D::MaxRowsAtCompileTime; r += 1)
      for (std::size_t c = 0; c < T2D::MaxColsAtCompileTime; c += 1) {
        t2d(r, c) = other()[r * T2D::MaxColsAtCompileTime + c];
      }
    return t2d;
  }

  template<class T2D>
  static void to_2d(const T2D &m, cv::FileStorage &other)
  {
    other << "[";
    for (std::size_t r = 0; r < T2D::MaxRowsAtCompileTime; r += 1)
      for (std::size_t c = 0; c < T2D::MaxColsAtCompileTime; c += 1) {
        other << m(r, c);
      }
    other << "]";
  }

  static std::string string_from_int32(int32_t val)//
  { return std::to_string(val); } //
  static std::string string_from_uint32(uint32_t val)//
  { return std::to_string(val); } //
//  static std::string string_from_int64(int64_t val)//
//  { return std::to_string(val); } //
  static std::string string_from_uint64(uint64_t val)//
  { return std::to_string(val); } //

  static int32_t int32_from_string(const std::string &str) //
  { return std::stol(str); } //
  static uint32_t uint32_from_string(const std::string &str) //
  { return std::stoul(str); } //
//  static int64_t int64_from_string(const std::string &str) //
//  { return std::stoll(str); } //
  static uint64_t uint64_from_string(const std::string &str) //
  { return std::stoull(str); } //



// ==============================================================================
// from/to methods
// ==============================================================================

  template<>
  Translate2 Translate2::from<FileStorageContext::Node>(FileStorageContext::Node &other)
  {
    return Translate2{from_1d<MuVector>(other)};
  }

  template<>
  void Translate2::to<cv::FileStorage>(cv::FileStorage &other) const
  {
    to_1d(t_, other);
  }

  template<>
  Translate2::CovarianceMatrix Translate2::cov_from<FileStorageContext::Node>(FileStorageContext::Node &other)
  {
    return from_2d<Translate2::CovarianceMatrix>(other);
  }

  template<>
  void Translate2::cov_to<cv::FileStorage>(const CovarianceMatrix &cov, cv::FileStorage &other)
  {
    to_2d(cov, other);
  }

  template<>
  Translate3 Translate3::from<FileStorageContext::Node>(FileStorageContext::Node &other)
  {
    return Translate3{from_1d<MuVector>(other)};
  }

  template<>
  void Translate3::to<cv::FileStorage>(cv::FileStorage &other) const
  {
    to_1d(t_, other);
  }

  template<>
  Rotate3 Rotate3::from<FileStorageContext::Node>(FileStorageContext::Node &other)
  {
    return Rotate3::RzRyRx(
      double(other()[0]),
      double(other()[1]),
      double(other()[2]));
  }

  template<>
  void Rotate3::to<cv::FileStorage>(cv::FileStorage &other) const
  {
    to_1d(xyz(), other);
  }

  template<>
  Transform3 Transform3::from<FileStorageContext::Node>(FileStorageContext::Node &other)
  {
    int is_valid = other()["is_valid"];
    if (!is_valid) {
      return Transform3{};
    }

    auto r_node = other()["r"];
    auto r_context = other.make(r_node);
    auto r = Rotate3::from(r_context);

    auto t_node = other()["t"];
    auto t_context = other.make(t_node);
    auto t = Translate3::from(t_context);

    return Transform3{r, t};
  }

  template<>
  void Transform3::to<cv::FileStorage>(cv::FileStorage &other) const
  {
    other << "{";
    other << "is_valid" << is_valid_;

    if (is_valid_) {
      other << "r";
      r_.to(other);
      other << "t";
      t_.to(other);
    }

    other << "}";
  }

  template<>
  Transform3::CovarianceMatrix Transform3::cov_from<FileStorageContext::Node>(FileStorageContext::Node &other)
  {
    return from_2d<Transform3::CovarianceMatrix>(other);
  }

  template<>
  void Transform3::cov_to<cv::FileStorage>(const CovarianceMatrix &cov, cv::FileStorage &other)
  {
    to_2d(cov, other);
  }

  template<>
  Transform3WithCovariance Transform3WithCovariance::from<FileStorageContext::Node>(FileStorageContext::Node &other)
  {
    auto tf_node = other()["tf"];
    auto tf_context = other.make(tf_node);
    auto tf = Transform3::from(tf_context);

    int is_cov_valid = other()["is_cov_valid"];
    if (!is_cov_valid) {
      return Transform3WithCovariance{tf};
    }

    auto cov_node = other()["cov"];
    auto cov_context = other.make(cov_node);
    auto cov = Transform3::cov_from(cov_context);

    return Transform3WithCovariance{tf, cov};
  }

  template<>
  void Transform3WithCovariance::to<cv::FileStorage>(cv::FileStorage &other) const
  {
    other << "{";
    other << "tf";
    tf_.to(other);
    other << "is_cov_valid" << is_cov_valid_;

    if (is_cov_valid_) {
      other << "cov";
      Transform3::cov_to(cov_, other);
    }
    other << "}";
  }

  template<>
  CameraInfo CameraInfo::from<FileStorageContext::Node>(FileStorageContext::Node &other)
  {
    auto imager_frame_id = other()["imager_frame_id"].string();
    auto width = uint32_from_string(other()["width"].string());
    auto height = uint32_from_string(other()["height"].string());

    auto k_node = other()["k"];
    auto k_cxt = other.make(k_node);
    auto k = (CameraMatrix{}
      << k_cxt()[0], k_cxt()[2], k_cxt()[3],
      0.0, k_cxt()[1], k_cxt()[4],
      0.0, 0.0, 1.0).finished();

    auto d_node = other()["d"];
    auto d_cxt = other.make(d_node);
    auto d = (DistCoeffs{}
      << d_cxt()[0], d_cxt()[1],
      d_cxt()[2], d_cxt()[3],
      d_cxt()[4]).finished();

    auto tci_node = other()["t_camera_imager"];
    auto tci_cxt = other.make(tci_node);
    auto tci = Transform3::from(tci_cxt);

    return CameraInfo{imager_frame_id, width, height, k, d, tci};
  }

  template<>
  void CameraInfo::to<cv::FileStorage>(cv::FileStorage &other) const
  {
    other << "{";
    other << "imager_frame_id" << imager_frame_id_;
    other << "width" << string_from_uint32(width_);
    other << "height" << string_from_uint32(height_);
    other << "k" << "["
          << camera_matrix_(0, 0) << camera_matrix_(1, 1)
          << camera_matrix_(0, 1)
          << camera_matrix_(0, 2) << camera_matrix_(1, 2) << "]";
    other << "d" << "["
          << dist_coeffs_(0) << dist_coeffs_(1)
          << dist_coeffs_(2) << dist_coeffs_(3)
          << dist_coeffs_(4) << "]";
    other << "t_camera_imager";
    t_camera_imager_.to(other);
    other << "}";
  }

  template<>
  CameraInfoMap CameraInfoMap::from<FileStorageContext::Node>(FileStorageContext::Node &other)
  {
    auto camera_info_map = CameraInfoMap{};

    auto ci_node = other()["camera_infos"];
    for (auto it = ci_node.begin(); it != ci_node.end(); ++it) {
      auto ci_context = other.make(*it);
      auto ci = CameraInfo::from(ci_context);
      camera_info_map.m_mutable().emplace(ci.imager_frame_id(), ci);
    }

    return camera_info_map;
  }

  template<>
  void CameraInfoMap::to<cv::FileStorage>(cv::FileStorage &other) const
  {
    other << "{";
    other << "camera_infos" << "[";
    for (auto &ci_pair : m_) {
      ci_pair.second.to(other);
    }
    other << "]";
    other << "}";
  }

  template<>
  Marker Marker::from<FileStorageContext::Node>(FileStorageContext::Node &other)
  {
    // There are two formats for the serialization of markers. The original format
    // writes the ID as an integer.

    auto id_node = other()["id"];
    if (id_node.type() == cv::FileNode::INT) {
      int id = int(id_node);
      int fixed = int(other()["f"]);

      auto t_node = other()["xyz"];
      auto t_context = other.make(t_node);
      auto t = Translate3::from(t_context);

      auto r_node = other()["rpy"];
      auto r_context = other.make(r_node);
      auto r = Rotate3::from(r_context);

      return Marker{std::uint64_t(id), Transform3WithCovariance{Transform3{r, t}}, fixed != 0};
    }

    // Otherwise use the hierarchical format
    std::uint64_t id = uint64_from_string(id_node.string());
    int fixed = int(other()["f"]);
    auto twc_node = other()["t_map_marker"];
    auto twc_context = other.make(twc_node);
    auto twc = Transform3WithCovariance::from(twc_context);

    return Marker{std::uint64_t(id), twc, fixed != 0};
  }

  template<>
  void Marker::to<cv::FileStorage>(cv::FileStorage &other) const
  {
    other << "{";
    other << "id" << string_from_uint64(id_);
    other << "f" << (is_fixed_ ? 1 : 0);
    other << "t_map_marker";
    t_map_marker_.to(other);
    other << "}";
  }

  template<>
  MapEnvironment MapEnvironment::from<FileStorageContext::Node>(FileStorageContext::Node &other)
  {
    auto description = other()["desc"].string();
    auto marker_dictionary_id = int(other()["dict"]);
    auto marker_length = double(other()["mlen"]);
    return MapEnvironment{description, marker_dictionary_id, marker_length};
  }

  template<>
  void MapEnvironment::to<cv::FileStorage>(cv::FileStorage &other) const
  {
    other << "{";
    other << "desc" << description_;
    other << "dict" << marker_dictionary_id_;
    other << "mlen" << marker_length_;
    other << "}";
  }

  template<>
  MarkerMap MarkerMap::from<FileStorageContext::Node>(FileStorageContext::Node &other)
  {
    MarkerMap map{};

    // The old format has the marker_length here
    auto ml_node = other()["marker_length"];
    if (!ml_node.empty()) {
      auto marker_length = double(ml_node);
      map = MarkerMap{MapEnvironment{"", 0, marker_length}};

    } else {
      // The new format has the marker_length in the map_environment
      auto me_node = other()["me"];
      auto me_context = other.make(me_node);
      auto me = MapEnvironment::from(me_context);
      map = MarkerMap{me};
    }

    auto markers_node = other()["markers"];
    for (auto it = markers_node.begin(); it != markers_node.end(); ++it) {
      auto marker_context = other.make(*it);
      auto marker = Marker::from(marker_context);
      map.add_marker(marker);
    }

    return map;
  }

  template<>
  void MarkerMap::to<cv::FileStorage>(cv::FileStorage &other) const
  {
    other << "{";

    other << "me";
    map_environment_.to(other);

    other << "markers" << "[";
    for (auto &marker : m_) {
      marker.second.to(other);
    }
    other << "]";
    other << "}";
  }

  template<>
  Observation Observation::from<FileStorageContext::Node>(FileStorageContext::Node &other)
  {
    auto is_valid = int(other()["is_valid"]) != 0;
    if (!is_valid) {
      return Observation{};
    }

    auto id = uint64_from_string(other()["id"].string());

    auto corners_node = other.make(other()["corners"]);
    auto corner0_node = other.make(corners_node()[0]);
    auto corner1_node = other.make(corners_node()[1]);
    auto corner2_node = other.make(corners_node()[2]);
    auto corner3_node = other.make(corners_node()[3]);
    Array corners_f_image{
      Translate2::from(corner0_node),
      Translate2::from(corner1_node),
      Translate2::from(corner2_node),
      Translate2::from(corner3_node),
    };

    auto cov_node = other.make(other()["cov"]);
    auto cov = Translate2::cov_from(cov_node);

    return Observation{id, corners_f_image, cov};
  }

  template<>
  void Observation::to<cv::FileStorage>(cv::FileStorage &other) const
  {
    other << "{";
    other << "is_valid" << is_valid_;
    if (is_valid()) {
      other << "id" << string_from_uint64(id_);
      other << "corners" << "[";
      corners_f_image_[0].to(other);
      corners_f_image_[1].to(other);
      corners_f_image_[2].to(other);
      corners_f_image_[3].to(other);
      other << "]";
      other << "cov";
      Element::cov_to(cov_, other);
    }
    other << "}";
  }

  template<>
  Stamp Stamp::from<FileStorageContext::Node>(FileStorageContext::Node &other)
  {
    std::int32_t s = int32_from_string(other()["s"].string());
    std::uint32_t ns = uint32_from_string(other()["ns"].string());
    return Stamp{s, ns};
  }

  template<>
  void Stamp::to<cv::FileStorage>(cv::FileStorage &other) const
  {
    other << "{";
    other << "s" << string_from_int32(sec_);
    other << "ns" << string_from_uint32(nanosec_);
    other << "}";
  }

  template<>
  Observations Observations::from<FileStorageContext::Node>(FileStorageContext::Node &other)
  {
    std::string imager_frame_id = other()["imager_frame_id"].string();
    Observations observations{imager_frame_id};

    auto observations_node = other()["observations"];
    for (auto it = observations_node.begin(); it != observations_node.end(); ++it) {
      auto observation_context = other.make(*it);
      auto observation = Observation::from(observation_context);
      observations.v_mutable().emplace_back(observation);
    }

    return observations;
  }

  template<>
  void Observations::to<cv::FileStorage>(cv::FileStorage &other) const
  {
    other << "{";

    other << "imager_frame_id" << imager_frame_id_;

    other << "observations" << "[";
    for (auto &observation : v_) {
      observation.to(other);
    }
    other << "]";

    other << "}";
  }

  template<>
  ObservationsSynced ObservationsSynced::from<FileStorageContext::Node>(FileStorageContext::Node &other)
  {
    auto stamp_node = other()["stamp"];
    auto stamp_context = other.make(stamp_node);
    auto stamp = Stamp::from(stamp_context);

    std::string camera_frame_id = other()["camera_frame_id"].string();

    auto observations_synced = ObservationsSynced{stamp, camera_frame_id};

    auto v_node = other()["v"];
    for (auto it = v_node.begin(); it != v_node.end(); ++it) {
      auto v_context = other.make(*it);
      auto v_item = Observations::from(v_context);
      observations_synced.v_mutable().emplace_back(v_item);
    }

    return observations_synced;
  }

  template<>
  void ObservationsSynced::to<cv::FileStorage>(cv::FileStorage &other) const
  {
    other << "{";

    other << "stamp";
    stamp_.to(other);

    other << "camera_frame_id" << camera_frame_id_;

    other << "v" << "[";
    for (auto &observations : v_) {
      observations.to(other);
    }
    other << "]";

    other << "}";
  }

  template<>
  ObservationsSeries ObservationsSeries::from<FileStorageContext::Node>(FileStorageContext::Node &other)
  {
    auto map_node = other()["marker_map"];
    auto map_context = other.make(map_node);
    auto map = MarkerMap::from(map_context);

    auto cim_node = other()["camera_info_map"];
    auto cim_context = other.make(cim_node);
    auto cim = CameraInfoMap::from(cim_context);

    auto observations_series = ObservationsSeries{map, cim};

    auto vector_node = other()["v"];
    for (auto it = vector_node.begin(); it != vector_node.end(); ++it) {
      auto observations_synced_context = other.make(*it);
      auto observations_synced = ObservationsSynced::from(observations_synced_context);
      observations_series.v_mutable().emplace_back(observations_synced);
    }

    return observations_series;
  }

  template<>
  void ObservationsSeries::to<cv::FileStorage>(cv::FileStorage &other) const
  {
    other << "{";
    other << "marker_map";
    map_.to(other);
    other << "camera_info_map";
    camera_info_map_.to(other);
    other << "v" << "[";

    for (auto &observations_synced : v_) {
      observations_synced.to(other);
    }

    other << "]";
    other << "}";
  }

// ==============================================================================
// MarkerMap save/load
// ==============================================================================

  void MarkerMap::save(const std::string &filename, Logger &logger) const
  {
    cv::FileStorage fs(filename, cv::FileStorage::WRITE | cv::FileStorage::FORMAT_YAML);
    if (!fs.isOpened()) {
      logger.error() << "Could not create MarkerMap file :" << filename;
      return;
    }

    fs << "marker_map";
    to(fs);
  }

  MarkerMap MarkerMap::load(const std::string &filename, Logger &logger)
  {
    cv::FileStorage fs(filename, cv::FileStorage::READ | cv::FileStorage::FORMAT_YAML);
    if (!fs.isOpened()) {
      logger.error() << "Could not open MarkerMap file :" << filename;
      return MarkerMap{};
    }

    FileStorageContext context{logger};
    auto root_node = context.make(fs.root());
    auto marker_map_node = root_node.make(root_node()["marker_map"]);

    auto map = MarkerMap::from(marker_map_node().empty() ? root_node : marker_map_node);
    return context.success() ? map : MarkerMap{};
  }


// ==============================================================================
// ObservationsSeries save/load
// ==============================================================================

  void ObservationsSeries::save(const std::string &filename, Logger &logger) const
  {
    cv::FileStorage fs(filename, cv::FileStorage::WRITE | cv::FileStorage::FORMAT_YAML);
    if (!fs.isOpened()) {
      logger.error() << "Could not create ObservationsSeries file :" << filename;
      return;
    }

    fs << "observations_series";
    to(fs);
  }

  ObservationsSeries ObservationsSeries::load(const std::string &filename, Logger &logger)
  {
    cv::FileStorage fs(filename, cv::FileStorage::READ | cv::FileStorage::FORMAT_YAML);
    if (!fs.isOpened()) {
      logger.error() << "Could not open ObservationsSeries file :" << filename;
      return ObservationsSeries{MarkerMap{}, CameraInfoMap{}};
    }

    FileStorageContext context{logger};
    auto root_node = context.make(fs.root());
    auto observations_series_node = root_node.make(root_node()["observations_series"]);

    auto os = ObservationsSeries::from(observations_series_node);
    return context.success() ? os : ObservationsSeries{MarkerMap{}, CameraInfoMap{}};
  }
}
