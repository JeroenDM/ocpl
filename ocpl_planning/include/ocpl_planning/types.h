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
typedef std::function<std::vector<JointPositions>(const Transform&, const JointPositions&)> IKFun;
typedef std::function<bool(const JointPositions&)> IsValidFun;

struct Vertice
{
    JointPositions q;
    size_t waypoint;
    double distance{ 0.0 };
    bool visited{ false };
    Vertice(const JointPositions& qi, size_t waypointi, double distancei)
      : q(qi), waypoint(waypointi), distance(distancei)
    {
    }
};
typedef std::shared_ptr<Vertice> VerticePtr;

inline const std::vector<VerticePtr>& getNeighbors(const std::vector<std::vector<VerticePtr>>& graph,
                                                   const VerticePtr& vertice)
{
    assert(vertice->waypoint + 1 < graph.size());
    return graph[vertice->waypoint + 1];
}

/** Kinematics, state validation and joint limits for a robot. **/
struct Robot
{
    size_t num_dof;
    size_t num_red_dof;
    JointLimits joint_limits;
    std::function<Transform(const JointPositions&)> fk;
    std::function<IKSolution(const Transform&, const JointPositions&)> ik;
    std::function<bool(const JointPositions&)> isValid;

    Robot() = delete;

    bool ok() const
    {
        return (!fk && !ik && !isValid && num_dof > 0 && joint_limits.size() == num_dof);
    }
};

struct Problem
{
    JointLimits redundant_joint_limits;
    std::function<Transform(const JointPositions&)> fk_fun;
    std::function<IKSolution(const Transform&, const JointPositions&)> ik_fun;
    std::function<bool(const JointPositions&)> is_valid_fun;
    std::function<double(const JointPositions&, const JointPositions&)> path_cost_fun;
    std::function<double(const TSR&, const JointPositions&)> state_cost_fun;
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
    LOCAL,
    // Orioli planner variations
    GREEDY,
    BIGREEDY,
    RRTLIKE
};
}  // namespace ocpl
