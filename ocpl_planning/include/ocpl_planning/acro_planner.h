#pragma once

#include <vector>
#include <functional>

#include <ocpl_graph/tree.h>
#include <ocpl_tsr/task_space_regions.h>
#include <ocpl_sampling/sampler.h>
#include <ocpl_planning/types.h>
#include <ocpl_planning/planner_base.h>

namespace ocpl
{
class UnifiedPlanner : public Planner
{
    PlannerSettings settings_;

    SamplerPtr q_red_local_sampler_;
    SamplerPtr q_sampler_;
    SamplerPtr tsr_sampler_;
    SamplerPtr tsr_local_sampler_;

  public:
    UnifiedPlanner(const Robot& robot, const PlannerSettings& settings);
    void initializeTaskSpaceSamplers(const std::vector<Bounds> tsr_bounds);

    std::vector<std::function<IKSolution()>> createGlobalWaypointSamplers(const std::vector<TSR>& task_space_regions,
                                                                          const JointLimits& redundant_joint_limits);

    /** \brief Global incremental sampling for all waypoints. **/
    std::vector<std::vector<JointPositions>>
    sampleGlobalIncremental(std::vector<std::function<IKSolution()>> path_samplers);

    /** \brief Sample on global uniform grid. **/
    std::vector<std::vector<JointPositions>> sampleGlobalGrid(std::vector<std::function<IKSolution()>> path_samplers);

    std::vector<JointPositions> sample(size_t waypoint, const JointPositions& q_bias,
                                       const std::vector<TSR>& task_space_regions);

    /** \brief Sample locally and incrementally, given a bias configuration. **/
    std::vector<JointPositions> sampleLocalIncremental(const JointPositions& q_bias,
                                                       std::function<IKSolution(const JointPositions&)> local_sampler);

    std::vector<std::vector<NodePtr>>
    createGlobalRoadmap(const std::vector<TSR>& task_space_regions, const JointLimits& redundant_joint_limits,
                        std::function<double(const TSR&, const JointPositions&)> state_cost_fun);

    Solution _solve(const std::vector<TSR>& task_space_regions, const JointLimits& redundant_joint_limits,
                    std::function<double(const JointPositions&, const JointPositions&)> path_cost_fun,
                    std::function<double(const TSR&, const JointPositions&)> state_cost_fun);
    Solution solve(const std::vector<TSR>& task) override;
};
}  // namespace ocpl
