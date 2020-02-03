
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
      return std::unique_ptr<Map>{};
    }
  };

// ==============================================================================
// SlamTask class
// ==============================================================================

  class SlamTask : public UpdateMapInterface
  {
    FiducialMath &fm_;
    std::unique_ptr<SlamTaskWork> stw_;


  public:
    SlamTask(FiducialMath &fm, const Map &empty_map) :
      UpdateMapInterface{}, fm_{fm}, stw_{std::make_unique<SlamTaskWork>(empty_map)}
    {}

    ~SlamTask() = default;

    void update_map(const Observations &observations,
                    const CameraInfo &camera_info,
                    Map &map) override
    {
      stw_->process_observations(observations, camera_info);
    }

    void update_map_for_publishing(Map &map) override
    {
      auto new_map = stw_->solve_map();
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
