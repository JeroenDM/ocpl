#pragma once

#include <vector>
#include <functional>

#include <ocpl_graph/tree.h>
#include <ocpl_tsr/task_space_regions.h>
#include <ocpl_planning/types.h>
#include <ocpl_planning/planner_base.h>

namespace ocpl
{
class UnifiedPlanner : public Planner
{
    PlannerSettings settings_;

  public:
    UnifiedPlanner(const Robot& robot, const PlannerSettings& settings)
      : Planner("unified_planner", robot), settings_(settings)
    {
    }

    /** \brief Global incremental sampling for all waypoints. **/
    std::vector<std::vector<JointPositions>>
    sampleGlobalIncremental(std::vector<std::function<IKSolution()>> path_samplers);

    /** \brief Sample on global uniform grid. **/
    std::vector<std::vector<JointPositions>> sampleGlobalGrid(std::vector<std::function<IKSolution()>> path_samplers);

    /** \brief Sample locally and incrementally, given a bias configuration. **/
    std::vector<JointPositions> sampleLocalIncremental(const JointPositions& q_bias,
                                                       std::function<IKSolution(const JointPositions&)> local_sampler);

    Solution _solve(const std::vector<TSR>& task_space_regions, const JointLimits& redundant_joint_limits,
                   std::function<double(const JointPositions&, const JointPositions&)> path_cost_fun,
                   std::function<double(const TSR&, const JointPositions&)> state_cost_fun);
    Solution solve(const std::vector<TSR>& task) override;
};
}  // namespace ocpl
