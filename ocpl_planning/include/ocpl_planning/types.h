#pragma once

#include <functional>
#include <iostream>
#include <string>
#include <vector>
#include <memory>

#include <ocpl_sampling/sampler.h>
#include <ocpl_tsr/task_space_regions.h>  // Bounds, Transform typedef, TSR
#include <ocpl_planning/math.h>

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

    /** \brief Solve inverse kinematics and only return solutions close to a bias config.
     *
     * Meaning non of the joints deviate more than `delta_q_max`.
     * **/
    JointPositions biasedIK(const Transform& tf, const JointPositions& q_red, const JointPositions& q_bias,
                            double delta_q_max) const
    {
        IKSolution sol = ik(tf, q_red);
        for (auto q_sol : sol)
        {
            if (normInfDiff(q_sol, q_bias) < delta_q_max)
            {
                return q_sol;
            }
        }
        return {};
    }

    /** Path validation using discrete linear interpolation.
     *
     * TODO: calculate resolution based on `delta_q_max`.
     * **/
    bool isPathValid(const JointPositions& q_from, const JointPositions& q_to, double /* delta_q_max */) const
    {
        if (!isValid(q_from))
            return false;
        // TODO figure out what they do mean in the paper
        // now fix the number of steps
        const int steps{ 3 };
        for (int step = 1; step < steps; ++step)
        {
            auto q_step = interpolate(q_from, q_to, static_cast<double>(step) / (steps - 1));
            if (!isValid(q_step))
            {
                return false;
            }
        }
        return true;
    }

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
