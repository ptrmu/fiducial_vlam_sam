
#include <chrono>
#include <iostream>
#include <iomanip>

#include "fiducial_vlam/fiducial_vlam.hpp"
#include "fiducial_vlam_msgs/msg/map.hpp"
#include "fiducial_vlam_msgs/msg/observations_synced.hpp"
#include "fvlam/build_marker_map_interface.hpp"
#include "fvlam/camera_info.hpp"
#include "fvlam/logger.hpp"
#include "fvlam/marker.hpp"
#include "fvlam/observation.hpp"
#include "fvlam/transform3_with_covariance.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "logger_ros2.hpp"
#include "vmap_context.hpp"


namespace fiducial_vlam
{
// ==============================================================================
// PsmContext class
// ==============================================================================

  struct PsmContext
  {
#undef PAMA_PARAM
#define PAMA_PARAM(n, t, d) PAMA_PARAM_DEFINE(n, t, d)
    PAMA_PARAMS_DEFINE(PSM_ALL_PARAMS)

    int timer_period_milliseconds_;
  };

// ==============================================================================
// BmmContext class
// ==============================================================================

  struct BmmContext
  {
#undef PAMA_PARAM
#define PAMA_PARAM(n, t, d) PAMA_PARAM_DEFINE(n, t, d)
    PAMA_PARAMS_DEFINE(BMM_ALL_PARAMS)
  };
}

namespace fvlam
{
// ==============================================================================
// BuildMarkerMapRecorderContext from method
// ==============================================================================

  template<>
  BuildMarkerMapRecorderContext BuildMarkerMapRecorderContext::from<fiducial_vlam::BmmContext>(
    fiducial_vlam::BmmContext &other)
  {
    return BuildMarkerMapRecorderContext{other.bmm_recorded_observations_name_};
  }

// ==============================================================================
// BuildMarkerMapTmmContext from method
// ==============================================================================

  template<>
  BuildMarkerMapTmmContext BuildMarkerMapTmmContext::from<fiducial_vlam::BmmContext>(
    fiducial_vlam::BmmContext &other, const MarkerMap &map_initial)
  {
    auto solve_tmm_factory = make_solve_tmm_factory(
      fvlam::SolveTmmContextCvSolvePnp{other.average_on_space_not_manifold_},
      map_initial.marker_length());

    return BuildMarkerMapTmmContext{
      solve_tmm_factory,
      other.bmm_tmm_try_shonan_,
      static_cast<fvlam::BuildMarkerMapTmmContext::NoiseStrategy>(other.bmm_tmm_noise_strategy_),
      other.bmm_tmm_fixed_sigma_r_,
      other.bmm_tmm_fixed_sigma_t_};
  }
}

namespace fiducial_vlam
{
// ==============================================================================
// BuildMarkerMapController class
// ==============================================================================

  class BuildMarkerMapController
  {
    rclcpp::Node &node_;
    fvlam::Logger &logger_;
    VmapDiagnostics &diagnostics_;
    BmmContext bmm_cxt_;
    int bmm_use_every_n_msg_;
    int bmm_cnt_every_n_msg_{0};
    std::unique_ptr<fvlam::MarkerMap> map_initial_;
    std::unique_ptr<fvlam::BuildMarkerMapInterface> bmm_interface_{};
    rclcpp::Subscription<fiducial_vlam_msgs::msg::ObservationsSynced>::SharedPtr sub_observations_{};
    bool pause_capture_{false};
    bool map_environment_inited{false};
    fvlam::MapEnvironment map_environment_{};

  public:
    BuildMarkerMapController(rclcpp::Node &node, fvlam::Logger &logger, VmapDiagnostics &diagnostics,
                             BmmContext bmm_cxt, const PsmContext &psm_cxt,
                             std::unique_ptr<fvlam::MarkerMap> map_initial) :
      node_{node}, logger_{logger}, diagnostics_{diagnostics},
      bmm_cxt_{std::move(bmm_cxt)}, bmm_use_every_n_msg_{std::max(1, bmm_cxt.bmm_use_every_n_msg_)},
      map_initial_{std::move(map_initial)}
    {
      // Instantiate a BuildMarkerMap class based on the parameter settings.
      switch (bmm_cxt_.bmm_algorithm_) {
        default:
        case 0: // record observations to file
          bmm_interface_ = make_build_marker_map(fvlam::BuildMarkerMapRecorderContext::from(bmm_cxt_),
                                                 logger_, *map_initial_);
          break;

        case 1: // t_marker0_marker1 techniques
          bmm_interface_ = make_build_marker_map(fvlam::BuildMarkerMapTmmContext::from(bmm_cxt_, *map_initial_),
                                                 logger_, *map_initial_);
          break;

        case 2: // isam_betweenfactor technique
          break;
      }

      if (!bmm_interface_) {
        return;
      }

      // Set up a subscriber for observations messages
      (void) sub_observations_;
      sub_observations_ = node_.create_subscription<fiducial_vlam_msgs::msg::ObservationsSynced>(
        psm_cxt.psm_sub_observations_topic_,
        rclcpp::QoS{rclcpp::ServicesQoS()},
        [this](fiducial_vlam_msgs::msg::ObservationsSynced::UniquePtr msg) -> void
        {
          // Check that these observations are from the same environment. We only work with
          // one environment at a time.
          auto map_environment = fvlam::MapEnvironment::from(msg->map_environment);
          if (!map_environment_inited) {
            map_environment_ = map_environment;
          }
          if (!map_environment_.equals(map_environment)) {
            logger_.error() << "Map Environment has changed - Ignoring message\n"
                            << "Expected: " << map_environment_.to_string() << "\n"
                            << "Actual: " << map_environment.to_string();
            return;
          }

          diagnostics_.sub_observations_count_ += 1;
          bmm_cnt_every_n_msg_ += 1;

          if (bmm_interface_ && !pause_capture_ && bmm_cnt_every_n_msg_ >= bmm_use_every_n_msg_) {
            diagnostics_.process_observations_count_ += 1;
            bmm_cnt_every_n_msg_ = 0;

            // From the observations message, pick out the CameraInfo and Observations
            auto camera_info_map = fvlam::CameraInfoMap::from(*msg);
            auto observations_synced = fvlam::ObservationsSynced::from(*msg);

            // Send these observations off for processing
            bmm_interface_->process(observations_synced, camera_info_map);
          }
        });
    }

    // Invoke the map builder method and report on the optimizattion errors
    std::unique_ptr<fvlam::MarkerMap> build()
    {
      if (!bmm_interface_) {
        return std::unique_ptr<fvlam::MarkerMap>{};
      }
      diagnostics_.build_count += 1;
      auto map = bmm_interface_->build();
      if (!map) {
        return std::unique_ptr<fvlam::MarkerMap>{};
      }

      // Log the errors. Have to test for the different builder types
      auto error_tmm = fvlam::BuildMarkerMapTmmContext::BuildError::from(*bmm_interface_, *map);
      if (error_tmm.valid_) {
        logger_.info() << error_tmm.to_string();
      }

      return map;
    }

    void pause()
    {
      if (bmm_interface_) {
        pause_capture_ = true;
      }
    }

    void resume()
    {
      if (bmm_interface_) {
        pause_capture_ = false;
      }
    }

    void finish()
    {
      if (bmm_interface_) {
        bmm_interface_.reset(nullptr);
      }
    }
  };

// ==============================================================================
// VmapNode class
// ==============================================================================

  class VmapNode : public rclcpp::Node
  {
    LoggerRos2 logger_;
    VmapDiagnostics diagnostics_;

    VmapContext cxt_{};
    PsmContext psm_cxt_{};
    BmmContext bmm_cxt_{};

    std::unique_ptr<BuildMarkerMapController> bmm_controller_{};
    std::unique_ptr<fvlam::MarkerMap> marker_map_{}; // Map that gets updated and published.
    rclcpp::Time exit_build_map_time_;

    // ROS publishers
    rclcpp::Publisher<fiducial_vlam_msgs::msg::Map>::SharedPtr pub_map_{};
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_visuals_{};
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_tf_{};

    rclcpp::TimerBase::SharedPtr map_pub_timer_{};

    void validate_parameters()
    {
      if (std::abs(psm_cxt_.psm_pub_map_frequency_hz_) < 1.e-10) {
        psm_cxt_.psm_pub_map_frequency_hz_ = 30. / 60.;
      }
      psm_cxt_.timer_period_milliseconds_ = static_cast<int>(1000. / psm_cxt_.psm_pub_map_frequency_hz_);
    }

    void setup_parameters()
    {
#undef PAMA_PARAM
#define PAMA_PARAM(n, t, d) PAMA_PARAM_INIT(n, t, d)
      PAMA_PARAMS_INIT((*this), cxt_, , VMAP_ALL_PARAMS, validate_parameters)
      PAMA_PARAMS_INIT((*this), psm_cxt_, , PSM_ALL_PARAMS, validate_parameters)
      PAMA_PARAMS_INIT((*this), bmm_cxt_, , BMM_ALL_PARAMS, validate_parameters)

#undef PAMA_PARAM
#define PAMA_PARAM(n, t, d) PAMA_PARAM_CHANGED(n, t, d)
      PAMA_PARAMS_CHANGED((*this), cxt_, , VMAP_ALL_PARAMS, validate_parameters, RCLCPP_INFO)
      PAMA_PARAMS_CHANGED((*this), psm_cxt_, , PSM_ALL_PARAMS, validate_parameters, RCLCPP_INFO)
      PAMA_PARAMS_CHANGED((*this), bmm_cxt_, , BMM_ALL_PARAMS, validate_parameters, RCLCPP_INFO)

#undef PAMA_PARAM
#define PAMA_PARAM(n, t, d) PAMA_PARAM_LOG(n, t, d)
      PAMA_PARAMS_LOG((*this), cxt_, , VMAP_ALL_PARAMS, RCLCPP_INFO)
      PAMA_PARAMS_LOG((*this), psm_cxt_, , PSM_ALL_PARAMS, RCLCPP_INFO)
      PAMA_PARAMS_LOG((*this), bmm_cxt_, , BMM_ALL_PARAMS, RCLCPP_INFO)

      // Check that all command line parameters are defined
#undef PAMA_PARAM
#define PAMA_PARAM(n, t, d) PAMA_PARAM_CHECK_CMDLINE(n, t, d)
      PAMA_PARAMS_CHECK_CMDLINE((*this), , VMAP_ALL_PARAMS PSM_ALL_PARAMS BMM_ALL_PARAMS, RCLCPP_ERROR)
    }

  public:
    explicit VmapNode(const rclcpp::NodeOptions &options) :
      Node{"vmap_node", options},
      logger_{*this},
      diagnostics_{now()},
      exit_build_map_time_{now()}
    {
      // Get parameters from the command line
      setup_parameters();

      // Initialize the map. Load from file or otherwise.
      marker_map_ = make_initial_marker_map(false);

      // ROS publishers.
      pub_map_ = create_publisher<fiducial_vlam_msgs::msg::Map>(
        psm_cxt_.psm_pub_map_topic_, 16);

      if (psm_cxt_.psm_pub_visuals_enable_) {
        pub_visuals_ = create_publisher<visualization_msgs::msg::MarkerArray>(
          psm_cxt_.psm_pub_visuals_topic_, 16);
      }

      if (psm_cxt_.psm_pub_tf_marker_enable_) {
        pub_tf_ = create_publisher<tf2_msgs::msg::TFMessage>("tf", 16);
      }

      // Timer for publishing map info
      map_pub_timer_ = create_wall_timer(
        std::chrono::milliseconds(psm_cxt_.timer_period_milliseconds_),
        [this]() -> void
        {
          timer_msg_callback();
        });

      (void) map_pub_timer_;
      logger_.info()
        << "To build a map of markers - set the map_cmd parameter." << std::endl
        << "  map_cmd start - Start capturing and incrementally processing images to build a map of markers.\n"
        << "  map_cmd pause - Stop capturing images but continue processing images already captured.\n"
        << "  map_cmd resume - Continue capturing images and processing them.\n"
        << "  map_cmd finish - Stop capturing and processing images. The last finished map is saved and the map building state is cleared.\n"
        << "  map_cmd diagnostics - Log diagnostics about the operation of vmap_node.";
      logger_.info() << "vmap_node ready.";
    }

  private:
    void timer_msg_callback()
    {
      // Publish only if there is a map. There might not
      // be a map if no markers have been observed.
      if (marker_map_) {
        rclcpp::Time enter_build_map_time{now()};
        // Give a little time for other tasks to process. For large optimizations, this timer routine
        // sucks up CPU cycles and other tasks are starved. Sometimes, the call to build_map takes longer than the
        // timer period so the timer_callback gets called again immediately when it exits. The following tests
        // attempt to prevent this a bit by skipping calls to build_map when the time between timer callbacks
        // is too short. The real solution is a separate thread for the long-running build_map routine.
        if ((enter_build_map_time - exit_build_map_time_) >=
            std::chrono::milliseconds(psm_cxt_.timer_period_milliseconds_) / 2) {
          build_marker_map();
        }
        exit_build_map_time_ = now();

        // Publish any map we have built to this point.
        publish_marker_map_and_visualization(now());
      }

      // Figure out if there is an map_cmd to process.
      if (!cxt_.map_cmd_.empty()) {
        std::string cmd{cxt_.map_cmd_};

        // Reset the cmd_string in preparation for the next command.
        PAMA_SET_PARAM((*this), cxt_, "", map_cmd, "");

        // Process the new command
        process_map_cmd(cmd);
      }
    }

    void build_marker_map()
    {
      if (bmm_controller_) {
        auto marker_map = bmm_controller_->build();
        if (marker_map) {
          // Have a new built map. log it.
          logger_.debug() << marker_map->to_string();

          // Save the map
          if (!cxt_.map_save_filename_.empty()) {
            marker_map->save(cxt_.map_save_filename_, logger_);
          }

          // Replace the current map
          marker_map_ = std::move(marker_map);
        }
      }
    }

    void publish_marker_map_and_visualization(const rclcpp::Time &stamp)
    {
      diagnostics_.pub_map_count_ += 1;
      auto header = std_msgs::msg::Header{}
        .set__stamp(stamp)
        .set__frame_id(psm_cxt_.psm_pub_map_frame_id_);

      // Create the map message and publish it. (always)
      auto msg = marker_map_->to<fiducial_vlam_msgs::msg::Map>()
        .set__header(header); // set header after setting map - TODO change Map msg to be stamped and have vector
      pub_map_->publish(msg);

      // Create and publish the marker visualization
      if (psm_cxt_.psm_pub_visuals_enable_) {
        diagnostics_.pub_visuals_count_ += 1;
        auto visuals_msg = visualization_msgs::msg::MarkerArray{};
        for (auto &id_marker_pair: *marker_map_) {
          auto marker_msg = id_marker_pair.second.to<visualization_msgs::msg::Marker>()
            .set__header(header); // set header after setting marker.
          visuals_msg.markers.emplace_back(marker_msg);
        }
        pub_visuals_->publish(visuals_msg);
      }

      // Create and publish the marker transform tree
      if (psm_cxt_.psm_pub_tf_marker_enable_) {
        diagnostics_.pub_tf_count_ += 1;
        tf2_msgs::msg::TFMessage tfs_msg;
        for (auto &id_marker_pair: *marker_map_) {
          auto &t_world_marker = id_marker_pair.second.t_map_marker().tf();

          std::ostringstream oss_child_frame_id;
          oss_child_frame_id << psm_cxt_.psm_pub_tf_marker_child_frame_id_
                             << std::setfill('0') << std::setw(3)
                             << id_marker_pair.first;

          auto tf_stamped_msg = geometry_msgs::msg::TransformStamped{}
            .set__header(header)
            .set__child_frame_id(oss_child_frame_id.str())
            .set__transform(t_world_marker.to<geometry_msgs::msg::Transform>());

          tfs_msg.transforms.emplace_back(tf_stamped_msg);
        }
        pub_tf_->publish(tfs_msg);
      }
    }

    void process_map_cmd(std::string &cmd)
    {
      // convert to ascii lower case
      for (auto &c : cmd) {
        c = std::tolower(c);
      }

      // Look for commands we understand
      if (cmd == "start") {
        if (bmm_controller_) {
          bmm_controller_.reset(nullptr);
        }
        bmm_controller_ = std::make_unique<BuildMarkerMapController>(
          *this, logger_, diagnostics_, bmm_cxt_, psm_cxt_,
          make_initial_marker_map(true));

      } else if (cmd == "pause") {
        if (bmm_controller_) {
          bmm_controller_->pause();
        }

      } else if (cmd == "resume") {
        if (bmm_controller_) {
          bmm_controller_->resume();
        }

      } else if (cmd == "finish") {
        if (bmm_controller_) {
          bmm_controller_->finish();
          bmm_controller_.reset(nullptr);
        }

      } else if (cmd == "diagnostics") {
        diagnostics_.report(logger_, now());

      } else {
        logger_.warn() << "Invalid command: " << cmd;
      }
    }

    // Copy markers from one map to another
    std::unique_ptr<fvlam::MarkerMap> copy_markers(const fvlam::MarkerMap &map_from_file,
                                                   bool copy_one_fixed_marker)
    {
      auto map = std::make_unique<fvlam::MarkerMap>(map_from_file.map_environment());

      for (auto &id_marker_pair : map_from_file) {
        if (!copy_one_fixed_marker ||
            id_marker_pair.second.is_fixed()) {
          map->add_marker(id_marker_pair.second);
          if (copy_one_fixed_marker) {
            return map;
          }
        }
      }

      return copy_one_fixed_marker ? std::unique_ptr<fvlam::MarkerMap>{} : std::move(map);
    }

    // Create a new MarkerMap from scratch.
    // If about to build a map
    //    If a filename specified and exists
    //      -> Load only the fixed markers from the file.
    //    If the filename is empty or the file doesn't exist
    //      -> Load one fixed marker based on the parameters
    // If not building a map
    //    If a filename specified and exists
    //      -> Load the complete map
    //    If the filename is empty or the file doesn't exist
    //      -> Load the fixed marker based on the parameters.
    std::unique_ptr<fvlam::MarkerMap> make_initial_marker_map(bool map_is_new_not_existing)
    {
      std::string &filename{cxt_.map_load_filename_};

      // if filename passed in then load the map from a file
      if (!filename.empty()) {
        logger_.info() << "Loading map file: " << filename;

        // load the map.
        auto map_from_file = fvlam::MarkerMap::load(filename, logger_);

        // If we successfully read the map, then return it or just the fixed nodes.
        if (map_from_file.marker_length() != 0.0) {
          auto map = copy_markers(map_from_file, map_is_new_not_existing);
          if (map) {
            return map;
          }
          logger_.error() << "Could not find fixed marker in map file.";
        } else {
          logger_.error() << "Could not load map file: " << filename;
        }
        logger_.error() << "Default to creating a map from parameters.";
      }

      // Create a map with one marker defined by the parameters.
//      auto map = std::make_unique<fvlam::MarkerMap>(cxt_.map_marker_length_); // Todo Fix this
      auto map = std::make_unique<fvlam::MarkerMap>();
      auto marker_new = fvlam::Marker(
        cxt_.map_init_id_, fvlam::Transform3WithCovariance{fvlam::Transform3{
          fvlam::Rotate3::RzRyRx(cxt_.map_init_pose_yaw_,
                                 cxt_.map_init_pose_pitch_,
                                 cxt_.map_init_pose_roll_),
          fvlam::Translate3{cxt_.map_init_pose_x_,
                            cxt_.map_init_pose_y_,
                            cxt_.map_init_pose_z_}}}, true);

      map->add_marker(std::move(marker_new));
      return map;
    }
  };

  std::shared_ptr<rclcpp::Node> vmap_node_factory(const rclcpp::NodeOptions &options)
  {
    return std::shared_ptr<rclcpp::Node>(new VmapNode(options));
  }

  void VmapDiagnostics::report(fvlam::Logger &logger, const rclcpp::Time &end_time)
  {
    double per_sec = 1.0 / (end_time - start_time_).seconds();
    logger.info() << "Received Observations: " << sub_observations_count_
                  << " (" << per_sec * sub_observations_count_ << " /sec)";
    logger.info() << "Processed Observations: " << process_observations_count_
                  << " (" << per_sec * process_observations_count_ << " /sec)";
    logger.info() << "Builds: " << build_count
                  << " (" << per_sec * build_count << " /sec)";
    logger.info() << "Published Maps: " << pub_map_count_
                  << " (" << per_sec * pub_map_count_ << " /sec)";
    logger.info() << "Published Visuals: " << pub_visuals_count_
                  << " (" << per_sec * pub_visuals_count_ << " /sec)";
    logger.info() << "Published Tfs: " << pub_tf_count_
                  << " (" << per_sec * pub_tf_count_ << " /sec)";

    sub_observations_count_ = 0;
    process_observations_count_ = 0;
    build_count = 0;
    pub_map_count_ = 0;
    pub_visuals_count_ = 0;
    pub_tf_count_ = 0;
    start_time_ = end_time;
  }
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(fiducial_vlam::VmapNode)
