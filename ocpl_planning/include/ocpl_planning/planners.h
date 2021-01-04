#pragma once

#include <vector>
#include <functional>

#include <ocpl_graph/tree.h>
#include <ocpl_tsr/task_space_regions.h>

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

}  // namespace ocpl
