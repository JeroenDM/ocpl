#pragma once

#include <vector>
#include <functional>

#include <ocpl_graph/tree.h>
#include <ocpl_tsr/task_space_regions.h>
#include <ocpl_planning/cost_functions.h>

namespace ocpl
{
typedef std::vector<double> JointPositions;
typedef std::vector<JointPositions> IKSolution;
typedef std::vector<std::vector<JointPositions>> GraphData;

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

std::vector<JointPositions> solve(const std::vector<TSR>& task_space_regions,
                                  std::function<IKSolution(const Transform&)> ik_fun,
                                  std::function<bool(const JointPositions&)> is_valid_fun,
                                  std::function<double(const JointPositions&, const JointPositions&)> path_cost_fun,
                                  std::function<double(const TSR&, const JointPositions&)> state_cost_fun);

}  // namespace ocpl
