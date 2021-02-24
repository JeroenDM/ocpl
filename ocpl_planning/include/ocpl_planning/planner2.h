#pragma once

#include <mutex>

#include <ocpl_planning/types.h>
#include <ocpl_planning/containers.h>
#include <ocpl_tsr/task_space_regions.h>
#include <ocpl_sampling/sampler.h>

namespace ocpl
{
/** Quick and dirty attempt at thread save logging.
 *
 * Only a single instance can exist at the time (google how to do singleton in C++).
 * **/
class TSLogger
{
    std::mutex mutex_;

  public:
    TSLogger() = default;
    ~TSLogger() = default;

    template <typename T>
    void log(const T& message)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        std::cout << message;
    }

    void logWaypoint(size_t waypoint, size_t num_samples)
    {
      std::unique_lock<std::mutex> lock(mutex_);
      std::cout << "ocpl_planner_2: processing path point " << waypoint;
      std::cout << " found " << num_samples << " samples\n";
    }
};

struct Planner2Settings
{
    std::string name{ "planner_2_default" };
    std::string method{ "local_stack" };
    double DQ_MAX{ 0.1 };   /** max joint motion **/
    double TSR_PERT{ 0.1 }; /** TSR perturbation for local sampling. **/
    size_t MAX_SHOTS{ 300 };
    size_t MIN_VALID_SAMPLES{ 500 };
    // std::string method{ "local_stack" };
};

class Planner2
{
  private:
    SamplerPtr q_red_local_sampler_;
    SamplerPtr q_sampler_;
    SamplerPtr tsr_sampler_;
    SamplerPtr tsr_local_sampler_;
    std::vector<int> has_tolerance_;

    Planner2Settings settings_;
    std::vector<TSR> task_;

    const std::string name_;
    mutable Robot robot_;

  public:
    Planner2(const std::string& name, const Robot& robot, const Planner2Settings& settings);
    ~Planner2() = default;

    void setTask(const std::vector<TSR>& task)
    {
        task_ = task;
    };
    void initializeTaskSpaceSamplers(const std::vector<Bounds> tsr_bounds);
    bool noColl(const JointPositions& q_from, const JointPositions& q_to) const;

    std::vector<JointPositions> sample(size_t waypoint, size_t num_samples) const;
    std::vector<JointPositions> sampleIncrementally(size_t waypoint, size_t min_num_samples, size_t batch_size) const;
    std::vector<JointPositions> sample(size_t waypoint, const JointPositions& q_bias, size_t num_samples) const;

    std::vector<std::vector<JointPositions>> createRoadMap() const;

    std::vector<JointPositions> updatePath(const std::vector<JointPositions>& path, const JointPositions& q, size_t k,
                                           size_t k_prev) const;
    std::vector<JointPositions> search(BaseContainer& A);
    std::vector<JointPositions> search_global(BaseContainer& A);

    Solution solve(const std::vector<TSR>& task);
};

}  // namespace ocpl
