#pragma once

#include <vector>
#include <functional>

#include <ocpl_graph/graph.h>
#include <ocpl_tsr/task_space_regions.h>
#include <ocpl_sampling/sampler.h>
#include <ocpl_planning/types.h>
#include <ocpl_planning/planner_base.h>

namespace ocpl
{
/** \brief Sample on global uniform grid. **/
std::vector<std::vector<JointPositions>> sampleGlobalGrid(std::vector<std::function<IKSolution()>> path_samplers);
/** \brief Global incremental sampling for all waypoints. **/
std::vector<std::vector<JointPositions>> sampleGlobalIncremental(std::vector<std::function<IKSolution()>> path_samplers,
                                                                 const int min_valid_samples, const int max_iters);

std::vector<JointPositions> sampleLocalGrid(const JointPositions& q_bias,
                                            std::function<IKSolution(const JointPositions&)> local_sampler);

/** \brief Sample locally and incrementally, given a bias configuration. **/
std::vector<JointPositions> sampleLocalIncremental(const JointPositions& q_bias,
                                                   std::function<IKSolution(const JointPositions&)> local_sampler,
                                                   const int min_valid_samples, const int max_iters);

class UnifiedPlanner : public Planner
{
  public:
    UnifiedPlanner(const Robot& robot, const PlannerSettings& settings) : Planner(robot, settings)
    {
    }
    std::vector<std::function<IKSolution()>> createGlobalWaypointSamplers(const std::vector<TSR>& task_space_regions,
                                                                          const JointLimits& redundant_joint_limits);

    std::vector<JointPositions> sample(size_t waypoint, const JointPositions& q_bias,
                                       const std::vector<TSR>& task_space_regions);

    std::vector<std::vector<ocpl_graph::NodePtr>>
    createGlobalRoadmap(const std::vector<TSR>& task_space_regions, const JointLimits& redundant_joint_limits,
                        std::function<double(const TSR&, const JointPositions&)> state_cost_fun);

    Solution _solve(const std::vector<TSR>& task_space_regions, const JointLimits& redundant_joint_limits,
                    std::function<double(const JointPositions&, const JointPositions&)> path_cost_fun,
                    std::function<double(const TSR&, const JointPositions&)> state_cost_fun);
    Solution solve(const std::vector<TSR>& task) override;

    Solution solve(const std::vector<TSR>& task,
                   std::function<double(const JointPositions&, const JointPositions&)> path_cost_fun,
                   std::function<double(const TSR&, const JointPositions&)> state_cost_fun) override;
};
}  // namespace ocpl
