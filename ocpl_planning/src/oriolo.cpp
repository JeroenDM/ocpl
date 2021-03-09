#include <ocpl_planning/oriolo.h>

#include <stdlib.h>
#include <cmath>
#include <algorithm>
#include <queue>

#include <ocpl_planning/cost_functions.h>
#include <ocpl_sampling/random_sampler.h>
#include <ocpl_planning/factories.h>
#include <ocpl_planning/math.h>

namespace graph
{
}

namespace ocpl
{
namespace oriolo
{
JointPositions OrioloPlanner::sampleIK(const TSR& tsr, const JointPositions& q_red, const JointPositions& q_bias)
{
    assert(tsr_local_sampler_ != nullptr);

    std::vector<double> v_prev = tsr.poseToValues(robot_.fk(q_bias));
    std::vector<double> v_bias = tsr_local_sampler_->getSample();

    // assume the variable that changes between two poses has no tolerance
    // (in most cases this is the x position, as the x-axis is oriented tangent to the path)
    assert(v_prev.size() == v_bias.size());
    const auto tsr_bounds = tsr.bounds.asVector();
    for (size_t dim{ 0 }; dim < v_prev.size(); ++dim)
    {
        if (tsr_bounds[dim].lower != tsr_bounds[dim].upper)
            v_bias[dim] += v_prev[dim];
    }

    return biasedIK(tsr.valuesToPose(v_bias), q_red, q_bias);
}


std::vector<JointPositions> OrioloPlanner::step(size_t start_index, size_t stop_index, const JointPositions& q_start,
                                                const std::vector<TSR>& task)
{
    assert(start_index >= 0);
    assert(start_index < stop_index);
    assert(stop_index < task.size());

    JointPositions q_current = q_start;
    JointPositions q_new;

    // add the start configurations already to the path
    std::vector<JointPositions> path{ q_start };

    // outer loop over the different points along the path
    for (size_t j = start_index; j < stop_index; ++j)
    {
        size_t l{ 0 };
        bool success{ false };
        // inner loop that tries to find a valid new configurations at j+1
        while (l < settings_.max_iters && !success)
        {
            // q_new = randConf(task.at(j + 1), q_current);
            q_new = sample(task[j + 1], q_current);
            if (!q_new.empty() && isPathValid(q_current, q_new))
            {
                path.push_back(q_new);
                success = true;
            }
            l++;
        }  // end inner loop

        // give up if this is reached for any path point
        if (l == settings_.max_iters)
        {
            // TODO add backtracking here?
            // j--; j >= start_index
            return {};
        }
        else
        {
            q_current = q_new;
        }
    }  // end outer loop
    return path;
}

std::vector<JointPositions> OrioloPlanner::greedy(const std::vector<TSR>& task)
{
    std::cout << "Started Oriolo greedy planner\n";

    std::vector<JointPositions> path;
    size_t iters{ 0 };
    bool success{ false };
    while (iters < settings_.max_iters && !success)
    {
        JointPositions q_start = sample(task[0]);
        if (!q_start.empty())
        {
            std::cout << "Found a good start configurations at iter: " << iters << "\n";
            path = step(0, task.size() - 1, q_start, task);
            if (!path.empty())
            {
                std::cout << "Solution found after iters: " << iters << "\n";
                success = true;
            }
        }
        iters++;
    }
    if (success)
        return path;
    else
        return {};
}

std::vector<JointPositions> OrioloPlanner::bidirectionalGreedy(const std::vector<TSR>& task)
{
    std::cout << "Started Oriolo bidirectional greedy planner\n";

    std::vector<TSR> reversed_task = task;
    std::reverse(reversed_task.begin(), reversed_task.end());

    std::vector<JointPositions> path;
    size_t iters{ 0 };
    bool success{ false };
    while (iters < settings_.max_iters && !success)
    {
        JointPositions q_start = sample(task[0]);
        if (!q_start.empty())
        {
            std::cout << "Found a good start configurations at iter: " << iters << "\n";
            path = step(0, task.size() - 1, q_start, task);
            if (!path.empty())
            {
                std::cout << "Solution found after iters: " << iters << "\n";
                success = true;
            }
        }

        if (!success)
        {
            JointPositions q_start = sample(reversed_task[0]);
            if (!q_start.empty())
            {
                std::cout << "Found a good reversed start configurations at iter: " << iters << "\n";
                path = step(0, reversed_task.size() - 1, q_start, reversed_task);
                if (!path.empty())
                {
                    std::cout << "Reversed solution found after iters: " << iters << "\n";
                    std::reverse(path.begin(), path.end());
                    success = true;
                }
            }
        }
        iters++;
    }
    if (success)
        return path;
    else
        return {};
}

std::pair<bool, graph::NodeData> OrioloPlanner::extend(const std::vector<ocpl::TSR>& task, graph::Tree& tree)
{
    auto q_rand = q_sampler_->getSample();
    auto n_near = getNear(q_rand, tree);
    auto q_extend = interpolate(n_near->data.q, q_rand, EXTEND_STEP_);

    // only use redundant joint from extended config
    JointPositions q_ext_red(q_extend.begin(), q_extend.begin() + robot_.num_red_dof);

    // solve inverse kinematics of the next waypoint with the nearest node as bias
    size_t next_waypoint = n_near->data.waypoint + 1;
    auto q_new = sampleIK(task[next_waypoint], q_ext_red, n_near->data.q);

    if (!q_new.empty() && isPathValid(n_near->data.q, q_new))
    {
        graph::NodePtr new_node = std::make_shared<graph::Node>(graph::NodeData{ q_new, next_waypoint });
        new_node->parent = n_near;
        tree[new_node] = {};
        tree[n_near].push_back(new_node);
        return { true, n_near->data };
    }
    else
    {
        return { false, {} };
    }
}

graph::NodePtr OrioloPlanner::getNear(const JointPositions& q, graph::Tree& tree)
{
    graph::NodePtr n_near;
    double min_dist = std::numeric_limits<double>::max();
    for (auto& pi : tree)
    {
        JointPositions qi = pi.first->data.q;
        double dist = L2NormDiff2(q, qi);
        if (dist < min_dist)
        {
            min_dist = dist;
            n_near = pi.first;
            // std::cout << "found nearest node: " << n_near->data.q.at(0.0);
            // std::cout << "  with distance: " << min_dist << "\n";
        }
    }
    return n_near;
}

std::vector<JointPositions> OrioloPlanner::rrtLike(const std::vector<ocpl::TSR>& task)
{
    size_t goal_waypoint = task.size() - 1;
    size_t current_waypoint{ 0 };
    size_t iters{ 0 };

    graph::Tree tree;

    // try to build a tree to the goal waypoint
    while (current_waypoint != goal_waypoint && iters < settings_.max_iters)
    {
        tree.clear();

        // create a tree from a random start configurations
        auto q0 = sample(task.at(0));
        if (q0.empty())
        {
            continue;
        }
        std::cout << "Found start config at iteration: " << iters << "\n";
        auto n0 = std::make_shared<graph::Node>(graph::NodeData{ q0, 0 });
        tree[n0] = {};

        size_t extend_iters{ 0 };
        do
        {
            auto res = extend(task, tree);
            if (res.first)
            {
                current_waypoint = res.second.waypoint;
                // here to extend step could be added for the other variations
            }
            extend_iters++;
        } while (current_waypoint != goal_waypoint && extend_iters < settings_.max_iters);

        std::cout << "tree reached waypoint: " << current_waypoint << " after " << extend_iters << "\n";

        iters++;
    }

    std::vector<JointPositions> path;
    if (current_waypoint == goal_waypoint)
    {
        std::cout << "rrtLike found a solution\n";
        graph::NodePtr goal;
        for (auto p : tree)
        {
            if (p.first->data.waypoint == goal_waypoint)
                goal = p.first;
        }
        graph::NodePtr current = goal;
        while (current->data.waypoint > 0)
        {
            if (current)
            {
                path.push_back(current->data.q);
                current = current->parent;
            }
            else
            {
                std::cout << "stuck in graph seach.\n";
                break;
            }
        }
        if (current)
        {
            path.push_back(current->data.q);
            std::reverse(path.begin(), path.end());
        }
    }
    else
    {
        std::cout << "rrtLike only reached waypoint: " << current_waypoint;
    }
    return path;
}

Solution OrioloPlanner::solve(const std::vector<ocpl::TSR>& task)
{
    initializeTaskSpaceSamplers(task.at(0).bounds.asVector());
    Solution sol;
    if (settings_.type == PlannerType::GREEDY)
    {
        sol.path = greedy(task);
        sol.success = sol.path.size() == task.size();
    }
    else if (settings_.type == PlannerType::BIGREEDY)
    {
        sol.path = bidirectionalGreedy(task);
        sol.success = sol.path.size() == task.size();
    }
    else if (settings_.type == PlannerType::RRTLIKE)
    {
        sol.path = rrtLike(task);
        sol.success = sol.path.size() == task.size();
    }
    else
    {
        throw std::invalid_argument("Invalid planning type for Oriolo planner");
    }
    return sol;
}

Solution OrioloPlanner::solve(const std::vector<TSR>& task,
                              std::function<double(const JointPositions&, const JointPositions&)> /* path_cost_fun */,
                              std::function<double(const TSR&, const JointPositions&)> /* state_cost_fun */)
{
    return solve(task);
}

}  // namespace oriolo
}  // namespace ocpl
