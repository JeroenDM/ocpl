#pragma once

#include <ocpl_planning/types.h>
#include <ocpl_planning/containers.h>
#include <ocpl_tsr/task_space_regions.h>
#include <ocpl_sampling/sampler.h>

namespace ocpl
{
struct Planner2Settings
{
    std::string name{ "planner_2_default" };
    std::string method{ "local_stack" };
    double DQ_MAX{ 0.1 };   /** max joint motion **/
    double TSR_PERT{ 0.1 }; /** TSR perturbation for local sampling. **/
    size_t MAX_SHOTS{ 300 };
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
    Robot robot_;

  public:
    Planner2(const std::string& name, const Robot& robot, const Planner2Settings& settings);
    ~Planner2() = default;

    void setTask(const std::vector<TSR>& task)
    {
        task_ = task;
    };
    void initializeTaskSpaceSamplers(const std::vector<Bounds> tsr_bounds);
    bool noColl(const JointPositions& q_from, const JointPositions& q_to);

    std::vector<JointPositions> sample(size_t waypoint, size_t num_samples);
    std::vector<JointPositions> sample(size_t waypoint, const JointPositions& q_bias, size_t num_samples);

    std::vector<JointPositions> updatePath(const std::vector<JointPositions>& path, const JointPositions& q, size_t k,
                                           size_t k_prev);
    std::vector<JointPositions> search(BaseContainer& A);
    // std::vector<JointPositions> search_global(BaseContainer& A);

    Solution solve(const std::vector<TSR>& task);
};

}  // namespace ocpl
