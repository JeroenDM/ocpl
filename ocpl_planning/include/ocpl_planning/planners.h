#pragma once

#include <vector>
#include <functional>

#include <ocpl_graph/tree.h>
#include <ocpl_tsr/task_space_regions.h>
#include <ocpl_planning/cost_functions.h>
#include <ocpl_planning/types.h>

namespace ocpl
{
class Planner
{
  protected:
    const std::string name_;
    Robot robot_;

  public:
    Planner(const std::string& name, const Robot& robot);
    ~Planner() = default;

    virtual Solution solve(const std::vector<TSR>& task) = 0;
};

/** \brief Global incremental sampling for all waypoints. **/
std::vector<std::vector<JointPositions>>
sampleGlobalIncremental(std::vector<std::function<IKSolution()>> path_samplers, const PlannerSettings& settings);

/** \brief Sample on global uniform grid. **/
std::vector<std::vector<JointPositions>> sampleGlobalGrid(std::vector<std::function<IKSolution()>> path_samplers);

Solution solve(const std::vector<TSR>& task_space_regions, const JointLimits& redundant_joint_limits,
               std::function<IKSolution(const Transform&, const JointPositions&)> ik_fun,
               std::function<bool(const JointPositions&)> is_valid_fun,
               std::function<double(const JointPositions&, const JointPositions&)> path_cost_fun,
               std::function<double(const TSR&, const JointPositions&)> state_cost_fun, PlannerSettings settings);

}  // namespace ocpl
