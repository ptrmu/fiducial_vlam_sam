
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "cv_utils.hpp"
#include "fiducial_math.hpp"
#include "map.hpp"
#include "observation.hpp"
#include "vmap_context.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "yaml-cpp/yaml.h"

#include <iostream>
#include <iomanip>
#include <fstream>

namespace fiducial_vlam
{

// ==============================================================================
// PsmContext class
// ==============================================================================

  struct PsmContext
  {
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)
    PSM_ALL_PARAMS

    int timer_period_milliseconds_;
  };

// ==============================================================================
// ToYAML class
// ==============================================================================

  class ToYAML
  {
    const Map &map_;
    YAML::Emitter emitter_{};

    void do_header()
    {
      emitter_ << YAML::Key << "marker_length" << YAML::Value << map_.marker_length();
      emitter_ << YAML::Key << "map_style" << YAML::Value << map_.map_style();
    }

    void do_marker(const Marker &marker)
    {
      emitter_ << YAML::BeginMap;
      emitter_ << YAML::Key << "id" << YAML::Value << marker.id();
      emitter_ << YAML::Key << "u" << YAML::Value << marker.update_count();
      emitter_ << YAML::Key << "f" << YAML::Value << (marker.is_fixed() ? 1 : 0);
      auto &c = marker.t_map_marker().transform().getOrigin();
      emitter_ << YAML::Key << "xyz" << YAML::Value << YAML::Flow
               << YAML::BeginSeq << c.x() << c.y() << c.z() << YAML::EndSeq;

      double roll, pitch, yaw;
      marker.t_map_marker().transform().getBasis().getRPY(roll, pitch, yaw);
      emitter_ << YAML::Key << "rpy" << YAML::Value << YAML::Flow
               << YAML::BeginSeq << roll << pitch << yaw << YAML::EndSeq;

      // Save the covariance if appropriate for the map_style
      if (map_.map_style() != Map::MapStyles::pose) {
        emitter_ << YAML::Key << "cov" << YAML::Value << YAML::Flow << YAML::BeginSeq;
        for (auto cov_element : marker.t_map_marker().cov()) {
          emitter_ << cov_element;
        }
        emitter_ << YAML::EndSeq;
      }

      emitter_ << YAML::EndMap;
    }

    void do_markers()
    {
      emitter_ << YAML::Key << "markers" << YAML::Value << YAML::BeginSeq;
      for (auto &marker_pair : map_.markers()) {
        auto &marker = marker_pair.second;
        do_marker(marker);
      }
      emitter_ << YAML::EndSeq;
    }

    void do_map()
    {
      emitter_ << YAML::BeginMap;
      do_header();
      do_markers();
      emitter_ << YAML::EndMap;
    }

  public:
    explicit ToYAML(const Map &map)
      : map_(map)
    {}

    void to_YAML(std::ostream &out_stream)
    {
      do_map();
      out_stream << emitter_.c_str() << std::endl;
    }
  };

  static std::string to_YAML_file(const std::unique_ptr<Map> &map, const std::string &filename)
  {
    std::ofstream out(filename);
    if (!out) {
      return std::string{"Config error: can not open config file for writing: "}.append(filename);
    }

    ToYAML{*map}.to_YAML(out);
    return std::string{};
  }

// ==============================================================================
// FromYAML class
// ==============================================================================

  class FromYAML
  {
    YAML::Node yaml_node_{};
    std::unique_ptr<Map> map_{};
    std::string error_msg_{};


    bool from_marker(YAML::Node &marker_node)
    {
      auto id_node = marker_node["id"];
      if (!id_node.IsScalar()) {
        return yaml_error("marker.id failed IsScalar()");
      }
      auto update_count_node = marker_node["u"];
      if (!update_count_node.IsScalar()) {
        return yaml_error("marker.update_count failed IsScalar()");
      }
      auto is_fixed_node = marker_node["f"];
      if (!is_fixed_node.IsScalar()) {
        return yaml_error("marker.is_fixed failed IsScalar()");
      }
      auto xyz_node = marker_node["xyz"];
      if (!xyz_node.IsSequence()) {
        return yaml_error("marker.xyz failed IsSequence()");
      }
      if (xyz_node.size() != 3) {
        return yaml_error("marker.xyz incorrect size");
      }
      auto rpy_node = marker_node["rpy"];
      if (!rpy_node.IsSequence()) {
        return yaml_error("marker.rpy failed IsSequence()");
      }
      if (rpy_node.size() != 3) {
        return yaml_error("marker.rpy incorrect size");
      }

      std::array<double, 3> xyz_data{};
      for (int i = 0; i < xyz_data.size(); i += 1) {
        auto i_node = xyz_node[i];
        if (!i_node.IsScalar()) {
          return yaml_error("marker.xyz[i] failed IsScalar()");
        }
        xyz_data[i] = i_node.as<double>();
      }
      std::array<double, 3> rpy_data{};
      for (int i = 0; i < rpy_data.size(); i += 1) {
        auto i_node = rpy_node[i];
        if (!i_node.IsScalar()) {
          return yaml_error("marker.rpy[i] failed IsScalar()");
        }
        rpy_data[i] = i_node.as<double>();
      }

      TransformWithCovariance::mu_type mu{
        xyz_data[0],
        xyz_data[1],
        xyz_data[2],
        rpy_data[0],
        rpy_data[1],
        rpy_data[2]};

      TransformWithCovariance::cov_type cov{{0.}};
      if (map_->map_style() != Map::MapStyles::pose) {
        auto cov_node = marker_node["cov"];
        if (!cov_node.IsSequence()) {
          return yaml_error("marker.cov failed IsSequence()");
        }
        if (cov_node.size() != 36) {
          return yaml_error("marker.cov incorrect size");
        }
        for (int i = 0; i < cov.size(); i += 1) {
          auto i_node = cov_node[i];
          if (!i_node.IsScalar()) {
            return yaml_error("marker.cov[i] failed IsScalar()");
          }
          cov[i] = i_node.as<double>();
        }
      }

      Marker marker(id_node.as<int>(), TransformWithCovariance(mu, cov));
      marker.set_is_fixed(is_fixed_node.as<int>());
      marker.set_update_count(update_count_node.as<int>());
      map_->add_marker(std::move(marker));
      return true;
    }

    bool from_markers(YAML::Node &markers_node)
    {
      for (YAML::const_iterator it = markers_node.begin(); it != markers_node.end(); ++it) {
        YAML::Node marker_node = *it;
        if (marker_node.IsMap()) {
          if (from_marker(marker_node)) {
            continue;
          }
          return false;
        }
        return yaml_error("marker failed IsMap()");
      }
      return true;
    }

    bool from_map()
    {
      if (yaml_node_.IsMap()) {
        Map::MapStyles map_style = Map::MapStyles::pose;
        auto map_style_node = yaml_node_["map_style"];
        if (map_style_node.IsScalar()) {
          map_style = static_cast<Map::MapStyles>(map_style_node.as<int>());
        }
        auto marker_length_node = yaml_node_["marker_length"];
        if (marker_length_node.IsScalar()) {
          auto marker_length = marker_length_node.as<double>();
          // create the map object now that we have the marker_length;
          map_ = std::make_unique<Map>(map_style, marker_length);
          auto markers_node = yaml_node_["markers"];
          if (markers_node.IsSequence()) {
            return from_markers(markers_node);
          }
          return yaml_error("markers failed IsSequence()");
        }
        return yaml_error("marker_length failed IsScalar()");
      }
      return yaml_error("root failed IsMap()");
    }

    bool yaml_error(const std::string &s)
    {
      error_msg_ = s;
      return false;
    }

  public:
    FromYAML() = default;

    std::string from_YAML(std::istream &in, std::unique_ptr<Map> &map)
    {
      error_msg_.clear();
      try {
        yaml_node_ = YAML::Load(in);
        if (from_map()) {
          map.swap(map_);
        }
      }
      catch (YAML::ParserException &ex) {
        error_msg_ = ex.what();
      }
      return error_msg_;
    }
  };

  static std::string from_YAML_file(const std::string &filename, std::unique_ptr<Map> &map)
  {
    std::ifstream in;
    in.open(filename, std::ifstream::in);
    if (!in.good()) {
      return std::string{"Config error: can not open config file for reading: "}.append(filename);
    }

    auto err_msg = FromYAML{}.from_YAML(in, map);
    if (!err_msg.empty()) {
      return std::string{"Config error: error parsing config file: "}
        .append(filename)
        .append(" error: ")
        .append(err_msg);
    }

    return err_msg; // no error
  }

// ==============================================================================
// VmapNode class
// ==============================================================================

  class VmapNode : public rclcpp::Node
  {
    VmapContext cxt_{};
    PsmContext psm_cxt_{};
    std::unique_ptr<BuildMarkerMapInterface> build_marker_map_{};

    std::unique_ptr<Map> map_{}; // Map that gets updated and published.
    std::uint64_t map_skip_images_count_{0};
    rclcpp::Time exit_build_map_time_;

    // ROS publishers
    rclcpp::Publisher<fiducial_vlam_msgs::msg::Map>::SharedPtr fiducial_map_pub_{};
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fiducial_markers_pub_{};
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_message_pub_{};

    rclcpp::Subscription<fiducial_vlam_msgs::msg::Observations>::SharedPtr observations_sub_{};
    rclcpp::TimerBase::SharedPtr map_pub_timer_{};

    void validate_parameters()
    {
      if (std::abs(psm_cxt_.psm_marker_map_publish_frequency_hz_) < 1.e-10) {
        psm_cxt_.psm_marker_map_publish_frequency_hz_ = 30. / 60.;
      }
      psm_cxt_.timer_period_milliseconds_ = static_cast<int>(1000. / psm_cxt_.psm_marker_map_publish_frequency_hz_);

      cxt_.map_init_transform_ = TransformWithCovariance(TransformWithCovariance::mu_type{
        cxt_.map_init_pose_x_, cxt_.map_init_pose_y_, cxt_.map_init_pose_z_,
        cxt_.map_init_pose_roll_, cxt_.map_init_pose_pitch_, cxt_.map_init_pose_yaw_});

      cxt_.map_skip_images_ = std::max(1, cxt_.map_skip_images_);
    }

    static void validate_fm_parameters()
    {}

    void setup_parameters()
    {
      // Do Vmap parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER((*this), cxt_, n, t, d)
      CXT_MACRO_INIT_PARAMETERS(VMAP_ALL_PARAMS, validate_parameters)

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(cxt_, n, t)
      CXT_MACRO_REGISTER_PARAMETERS_CHANGED((*this), VMAP_ALL_PARAMS, validate_parameters)

      // Do PubSub Map parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER((*this), psm_cxt_, n, t, d)
      CXT_MACRO_INIT_PARAMETERS(PSM_ALL_PARAMS, validate_parameters)

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(psm_cxt_, n, t)
      CXT_MACRO_REGISTER_PARAMETERS_CHANGED((*this), PSM_ALL_PARAMS, validate_parameters)

      // Display all the parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_SORTED_PARAMETER(cxt_, n, t, d)
      CXT_MACRO_LOG_SORTED_PARAMETERS(RCLCPP_INFO, get_logger(), "VmapNode Parameters", VMAP_ALL_PARAMS)

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_SORTED_PARAMETER(psm_cxt_, n, t, d)
      CXT_MACRO_LOG_SORTED_PARAMETERS(RCLCPP_INFO, get_logger(), "PubSub Map Parameters", PSM_ALL_PARAMS)

      // Check that all command line parameters are defined
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_CHECK_CMDLINE_PARAMETER(n, t, d)
      CXT_MACRO_CHECK_CMDLINE_PARAMETERS((*this), VMAP_ALL_PARAMS PSM_ALL_PARAMS)
    }

    // Special "initialize map from camera location" mode
    void initialize_map_from_observations(const Observations &observations,
                                          const CameraInfoInterface &ci)
    {
      // Find the marker with the lowest id
      int min_id = std::numeric_limits<int>::max();
      const Observation *min_obs{};
      for (auto &obs : observations.observations()) {
        if (obs.id() < min_id) {
          min_id = obs.id();
          min_obs = &obs;
        }
      }

      // Find t_camera_marker
      auto t_camera_marker = CvUtils::solve_t_camera_marker(*min_obs, ci, cxt_.map_marker_length_);

      // And t_map_camera
      auto t_map_camera = cxt_.map_init_transform_;

      // Figure t_map_marker and add a marker to the map.
      auto t_map_marker = TransformWithCovariance(t_map_camera.transform() * t_camera_marker.transform());
      map_->add_marker(Marker(min_id, std::move(t_map_marker)));
    }

  public:
    explicit VmapNode(const rclcpp::NodeOptions &options) :
      Node{"vmap_node", options},
      exit_build_map_time_{now()}
    {
      // Get parameters from the command line
      setup_parameters();

      // Initialize the map. Load from file or otherwise.
      map_ = initialize_map(cxt_.map_load_filename_, Map::MapStyles::pose);

      // ROS publishers.
      fiducial_map_pub_ = create_publisher<fiducial_vlam_msgs::msg::Map>(
        psm_cxt_.psm_fiducial_map_pub_topic_, 16);

      if (psm_cxt_.psm_publish_marker_visualizations_) {
        fiducial_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
          psm_cxt_.psm_fiducial_markers_pub_topic_, 16);
      }

      if (psm_cxt_.psm_publish_tfs_) {
        tf_message_pub_ = create_publisher<tf2_msgs::msg::TFMessage>("tf", 16);
      }

      // Timer for publishing map info
      map_pub_timer_ = create_wall_timer(
        std::chrono::milliseconds(psm_cxt_.timer_period_milliseconds_),
        [this]() -> void
        {
          timer_msg_callback();
        });

      (void) observations_sub_;
      (void) map_pub_timer_;
      RCLCPP_INFO(get_logger(), "To build a map of markers - set map_cmd parameter.");
      RCLCPP_INFO(get_logger(), "  map_cmd start - Start capturing and incrementally processing images to build a map of markers.");
      RCLCPP_INFO(get_logger(), "  map_cmd stop - Stop capturing images but continue processing images already captured.");
      RCLCPP_INFO(get_logger(), "  map_cmd continue - Continue capturing images and processing them.");
      RCLCPP_INFO(get_logger(), "  map_cmd done - Stop capturing and processing images. The last finished map is saved and the map building state is cleared.");
      RCLCPP_INFO(get_logger(), "vmap_node ready");
    }

  private:

    void timer_msg_callback()
    {
      // Publish only if there is a map. There might not
      // be a map if no markers have been observed.
      if (map_) {
        rclcpp::Time enter_build_map_time{now()};
        // Give a little time for other tasks to process. For large optimizations, this timer routine
        // sucks up CPU cycles and other tasks are starved.
        if ((enter_build_map_time - exit_build_map_time_) >=
            std::chrono::milliseconds(psm_cxt_.timer_period_milliseconds_) / 2) {
          build_map();
        }
        exit_build_map_time_ = now();

        // Publish any map we have built to this point.
        publish_map_and_visualization();
      }

      // Figure out if there is an update maps command to process.
      if (!cxt_.map_cmd_.empty()) {
        std::string cmd{cxt_.map_cmd_};

        // Reset the cmd_string in preparation for the next command.
        CXT_MACRO_SET_PARAMETER((*this), cxt_, map_cmd, "");

        auto ret_str = process_map_cmd(cmd);
        if (!ret_str.empty()) {
          RCLCPP_INFO(get_logger(), "UpdateMapCmd response: %s", ret_str.c_str());
        }
      }
    }

    void build_map()
    {
      if (build_marker_map_) {
        // Build a map from the observations
        auto ret_str = build_marker_map_->build_marker_map(*map_);
        if (!ret_str.empty()) {
          RCLCPP_INFO(get_logger(), "UpdateMap response: %s", ret_str.c_str());
        }

        // Save the map
        if (!cxt_.map_save_filename_.empty()) {
          auto err_msg = to_YAML_file(map_, cxt_.map_save_filename_);
          if (!err_msg.empty()) {
            RCLCPP_INFO(get_logger(), err_msg.c_str());
          }
        }
      }
    }

    void publish_map_and_visualization()
    {
      // publish the map
      std_msgs::msg::Header header;
      header.stamp = now();
      header.frame_id = psm_cxt_.psm_map_frame_id_;
      fiducial_map_pub_->publish(*map_->to_map_msg(header));

      // Publish the marker Visualization
      if (psm_cxt_.psm_publish_marker_visualizations_) {
        fiducial_markers_pub_->publish(to_marker_array_msg());
      }

      // Publish the transform tree
      if (psm_cxt_.psm_publish_tfs_) {
        tf_message_pub_->publish(to_tf_message());
      }
    }

    std::string process_map_cmd(std::string &cmd)
    {
      // Look for the start command
      if (cmd == "start") {

        // Subscribe to observations messages if we have not already.
        if (observations_sub_ == nullptr) {
          observations_sub_ = create_subscription<fiducial_vlam_msgs::msg::Observations>(
            psm_cxt_.psm_fiducial_observations_sub_topic_,
            512,
            [this](const fiducial_vlam_msgs::msg::Observations::UniquePtr msg) -> void
            {
              observations_msg_callback(msg);
            });
        }

        // Initialize the map to empty. Use the covariance style in this case because we
        // only have SAM map creators at this point. For SAM, it doesn't seem that
        // using the covariance of the discovered markers helps in locating the camera.
        map_ = initialize_map("", Map::MapStyles::pose);

        // Create a builder object. Now any observation messages will get passed to it.
        // Notice that a running map builder will get shut down here.
        build_marker_map_ = make_sam_build_marker_map(cxt_, *map_);

        // Pass the "start" command to the builder in-case it wants to do anything (like report status)
        return build_marker_map_->map_cmd(cmd);
      }

      // Reset the map building.
      if (cmd == "reset") {
        build_marker_map_.reset(nullptr);
        return std::string("Map building reset.");
      }

      // If we have and active map builder, then just pass the command along.
      if (build_marker_map_) {
        auto ret_str = build_marker_map_->map_cmd(cmd);
        return ret_str;
      }

      return std::string("No Update active");
    }

    void observations_msg_callback(const fiducial_vlam_msgs::msg::Observations::UniquePtr &msg)
    {
      // skip some messages so we don't get too overloaded doing the isam optimization.
      map_skip_images_count_ += 1;
      if ((map_skip_images_count_ % cxt_.map_skip_images_) != 0) {
        return;
      }

      // Create an CameraInfo. This object will get passed to the map builder.
      std::unique_ptr<const CameraInfoInterface> ci{make_camera_info(msg->camera_info)};

      // Get observations from the message. This object will also get passed to the map builder
      std::unique_ptr<Observations> observations{std::make_unique<Observations>(*msg)};

      // If the map has not yet been initialized, then initialize it with these observations.
      // This is only used for the special camera based map initialization
      if (!map_ && observations->size() > 0) {
        initialize_map_from_observations(*observations, *ci);
      }

      // There is nothing to do at this point unless we have more than one observation.
      if (observations->size() < 2) {
        return;
      }

      // Process these observations if we are building a map
      if (build_marker_map_) {
        build_marker_map_->process_observations(std::move(observations), std::move(ci));
      }
    }

    tf2_msgs::msg::TFMessage to_tf_message()
    {
      auto stamp = now();
      tf2_msgs::msg::TFMessage tf_message;

      for (auto &marker_pair: map_->markers()) {
        auto &marker = marker_pair.second;
        auto mu = marker.t_map_marker().mu();

        std::ostringstream oss_child_frame_id;
        oss_child_frame_id << psm_cxt_.psm_marker_prefix_frame_id_ << std::setfill('0') << std::setw(3) << marker.id();

        tf2::Quaternion q;
        q.setRPY(mu[3], mu[4], mu[5]);
        auto tf2_transform = tf2::Transform(q, tf2::Vector3(mu[0], mu[1], mu[2]));

        geometry_msgs::msg::TransformStamped msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = psm_cxt_.psm_map_frame_id_;
        msg.child_frame_id = oss_child_frame_id.str();
        msg.transform = tf2::toMsg(tf2_transform);

        tf_message.transforms.emplace_back(msg);
      }

      return tf_message;
    }

    visualization_msgs::msg::MarkerArray to_marker_array_msg()
    {
      visualization_msgs::msg::MarkerArray markers;
      for (auto &marker_pair: map_->markers()) {
        auto &marker = marker_pair.second;
        visualization_msgs::msg::Marker marker_msg;
        marker_msg.id = marker.id();
        marker_msg.header.frame_id = psm_cxt_.psm_map_frame_id_;
        marker_msg.pose = to_Pose_msg(marker.t_map_marker());
        marker_msg.type = visualization_msgs::msg::Marker::CUBE;
        marker_msg.action = visualization_msgs::msg::Marker::ADD;
        marker_msg.scale.x = 0.1;
        marker_msg.scale.y = 0.1;
        marker_msg.scale.z = 0.01;
        marker_msg.color.r = 1.f;
        marker_msg.color.g = 1.f;
        marker_msg.color.b = 0.f;
        marker_msg.color.a = 1.f;
        markers.markers.emplace_back(marker_msg);
      }
      return markers;
    }

    std::unique_ptr<Map> initialize_map(const std::string &map_load_filename,
                                        Map::MapStyles new_map_style)
    {
      std::unique_ptr<Map> map_unique{};

      // If not building a map, then load the map from a file
      if (!map_load_filename.empty()) {
        RCLCPP_INFO(get_logger(), "Loading map file '%s'", map_load_filename.c_str());

        // load the map.
        auto err_msg = from_YAML_file(map_load_filename, map_unique);

        if (err_msg.empty()) {
          return map_unique;
        }
        // If an error, fall into initialize the map
        RCLCPP_ERROR(get_logger(), err_msg.c_str());
        RCLCPP_ERROR(get_logger(), "Falling into initialize map. (style: %d)", cxt_.map_init_style_);
      }

      // Building a map. Use the different styles of map initialization.
      // If style 2, then need to wait for an observation for initialization.
      if (cxt_.map_init_style_ == 2) {
        return map_unique;
      }

      // if Style == 0, look for a file and pull the pose from it.
      // If there is a problem, fall into style 1.
      if (cxt_.map_init_style_ == 0) {
        std::unique_ptr<Map> map_temp{};
        auto err_msg = from_YAML_file(map_load_filename, map_temp);
        if (!err_msg.empty()) {
          RCLCPP_ERROR(get_logger(), "Error while trying to initialize map style 0");
          RCLCPP_ERROR(get_logger(), err_msg.c_str());
          RCLCPP_ERROR(get_logger(), "Falling into initialize map style 1");

        } else {
          auto marker_temp = map_temp->find_marker(cxt_.map_init_id_);
          if (marker_temp == nullptr) {
            RCLCPP_ERROR(get_logger(), "Error while trying to initialize map style 0");
            RCLCPP_ERROR(get_logger(), "Map file '%s' does not contain a marker with id %d",
                         map_load_filename.c_str(), cxt_.map_init_id_);
            RCLCPP_ERROR(get_logger(), "Falling into initialize map style 1");

          } else {
            auto marker_copy = *marker_temp;
            marker_copy.set_is_fixed(true);
            map_unique = std::make_unique<Map>(new_map_style, cxt_.map_marker_length_);
            map_unique->add_marker(std::move(marker_copy));
            return map_unique;
          }
        }
      }

      // Style 1 initialization. Get the info from parameters.
      map_unique = std::make_unique<Map>(new_map_style, cxt_.map_marker_length_);
      auto marker_new = Marker(cxt_.map_init_id_, cxt_.map_init_transform_);
      marker_new.set_is_fixed(true);
      map_unique->add_marker(std::move(marker_new));

      return map_unique;
    }
  };

  std::shared_ptr<rclcpp::Node> vmap_node_factory(const rclcpp::NodeOptions &options)
  {
    return std::shared_ptr<rclcpp::Node>(new VmapNode(options));
  }
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(fiducial_vlam::VmapNode)
