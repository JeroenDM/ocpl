#pragma once

#include <functional>
#include <iostream>
#include <string>
#include <vector>
#include <memory>

#include <ocpl_sampling/sampler.h>
#include <ocpl_tsr/task_space_regions.h>  // Bounds, Transform typedef, TSR

namespace ocpl
{
typedef std::vector<double> JointPositions;
typedef std::vector<JointPositions> IKSolution;
typedef std::vector<std::vector<JointPositions>> GraphData;

typedef std::vector<Bounds> JointLimits;

typedef std::function<Transform(const JointPositions&)> FKFun;
typedef std::function<IKSolution(const Transform&, const JointPositions&)> IKFun;
typedef std::function<bool(const JointPositions&)> IsValidFun;

/** Kinematics, state validation and joint limits for a robot. **/
struct Robot
{
    size_t num_dof;
    size_t num_red_dof;
    JointLimits joint_limits;
    FKFun fk;
    IKFun ik;
    IsValidFun isValid;

    /** Force users to initialize everyting explicitly.
     * Note that you can still assing nullptrs to functions.
     * */
    Robot() = delete;

    bool ok() const
    {
        return (!fk && !ik && !isValid && num_dof > 0 && joint_limits.size() == num_dof);
    }
};

/** Settings that are specifc to a robot and task combination. **/
struct CaseSettings
{
    std::vector<int> tsr_resolution{};
    std::vector<int> redundant_joints_resolution{};
};

struct Solution
{
    bool success;
    std::vector<JointPositions> path;
    double cost{ 0.0 };
};

enum class PlannerType
{
    // unified planner types
    GLOBAL,
    GLOBAL_DFS,
    LOCAL_DFS,
    LOCAL_BEST_FIRST_DFS,
    // Orioli planner variations
    GREEDY,
    BIGREEDY,
    RRTLIKE
};
}  // namespace ocpl
