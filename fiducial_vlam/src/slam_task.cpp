
#include "fiducial_math.hpp"

#include "observation.hpp"
#include "map.hpp"
#include "task_thread.hpp"

namespace fiducial_vlam
{

// ==============================================================================
// SlamTaskWork class
// ==============================================================================

  class SlamTaskWork
  {
    std::unique_ptr<Map> empty_map_;

  public:
    SlamTaskWork(const Map &empty_map) :
      empty_map_{std::make_unique<Map>(empty_map)}
    {}

    void process_observations(const Observations &observations,
                              const CameraInfo &camera_info)
    {
    }

    std::unique_ptr<Map> solve_map()
    {
      return std::make_unique<Map>(*empty_map_);
    }
  };

// ==============================================================================
// SlamTask class
// ==============================================================================

  class SlamTask : public UpdateMapInterface
  {
    FiducialMath &fm_;
    std::unique_ptr<SlamTaskWork> stw_;
    std::future<std::unique_ptr<Map>> solve_map_future_{};

  public:
    SlamTask(FiducialMath &fm, const Map &empty_map) :
      UpdateMapInterface{}, fm_{fm}, stw_{std::make_unique<SlamTaskWork>(empty_map)}
    {}

    ~SlamTask() = default;

    void update_map(const Observations &observations,
                    const CameraInfo &camera_info,
                    Map &map) override
    {
      auto func = [observations, camera_info](SlamTaskWork &stw) -> void
      {
        stw.process_observations(observations, camera_info);
      };
      func(*stw_);
    }

    void update_map_for_publishing(Map &map) override
    {
      // If the future is valid, then a map is being solved and we should check
      // to see if it is complete
      if (solve_map_future_.valid()) {

        // Is it complete?
        auto status = solve_map_future_.wait_for(std::chrono::milliseconds(0));
        if (status == std::future_status::ready) {
          auto new_map = solve_map_future_.get();
          map.reset(*new_map);
        }
        return;
      }

      // A map is not being solved, so queue a solution up.
      std::promise<std::unique_ptr<Map>> solve_map_promise{};
      solve_map_future_ = solve_map_promise.get_future();

      auto func = [promise = std::move(solve_map_promise)](SlamTaskWork &stw) mutable -> void
      {
        auto new_map = stw.solve_map();
        promise.set_value(std::move(new_map));
      };

      func(*stw_);
   }

    std::string update_map_cmd(std::string &cmd) override
    {
      return std::string{};
    }
  };

  std::unique_ptr<UpdateMapInterface> slam_task_factory(FiducialMath &fm, const Map &empty_map)
  {
    return std::unique_ptr<UpdateMapInterface>{new SlamTask{fm, empty_map}};
  }
}
