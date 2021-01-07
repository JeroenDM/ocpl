#pragma once

#include <vector>
#include <functional>

#include <ocpl_graph/tree.h>
#include <ocpl_tsr/task_space_regions.h>
#include <ocpl_planning/cost_functions.h>
#include <ocpl_planning/settings.h>

namespace ocpl
{
typedef std::vector<double> JointPositions;
typedef std::vector<JointPositions> IKSolution;
typedef std::vector<std::vector<JointPositions>> GraphData;

typedef std::vector<Bounds> JointLimits;

std::vector<JointPositions> sampleTSR(const TSR& tsr, std::function<bool(const JointPositions&)> is_valid,
                                      std::function<IKSolution(const Transform&)> generic_inverse_kinematics);

std::vector<JointPositions> findPath(const std::vector<TSR>& tsrs,
                                     std::function<double(const NodePtr, const NodePtr)> cost_function,
                                     std::function<double(const TSR&, const JointPositions&)> state_cost,
                                     std::function<bool(const JointPositions&)> is_valid,
                                     std::function<IKSolution(const TSR&)> generic_inverse_kinematics);

std::vector<JointPositions> findPath(const std::vector<std::function<IKSolution(int)>>& path_samplers,
                                     const std::vector<Transform>& nominal_tfs,
                                     std::function<double(const NodePtr, const NodePtr)> cost_function,
                                     std::function<double(const Transform&, const JointPositions&)> state_cost,
                                     std::function<bool(const JointPositions&)> is_valid);

std::vector<std::vector<JointPositions>>
createCSpaceGraphIncrementally(std::vector<std::function<IKSolution()>> path_samplers, const PlannerSettings& settings);

std::vector<std::vector<JointPositions>> createCSpaceGraphGrid(std::vector<std::function<IKSolution()>> path_samplers);

std::vector<JointPositions> solve(const std::vector<TSR>& task_space_regions, const JointLimits& redundant_joint_limits,
                                  std::function<IKSolution(const Transform&, const JointPositions&)> ik_fun,
                                  std::function<bool(const JointPositions&)> is_valid_fun,
                                  std::function<double(const JointPositions&, const JointPositions&)> path_cost_fun,
                                  std::function<double(const TSR&, const JointPositions&)> state_cost_fun,
                                  PlannerSettings settings);

}  // namespace ocpl
