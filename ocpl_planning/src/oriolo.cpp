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
OrioloPlanner::OrioloPlanner(const std::string& name, const Robot& robot, const OrioloSpecificSettings& settings)
  : Planner(name, robot), settings_(settings), EXTEND_STEP_(settings_.D * std::sqrt(robot.num_dof))
{
    // sampler to generate perturbations on redundant joints with max deviation d around zero
    q_red_local_sampler_ = createLocalSampler(3, settings_.D, SamplerType::RANDOM);

    // sampler to generate completely random robot configurations inside the limits
    q_sampler_ = createIncrementalSampler(robot.joint_limits, SamplerType::RANDOM);
}

void OrioloPlanner::initializeTaskSpaceSamplers(const std::vector<Bounds> tsr_bounds)
{
    //  sampler to generate random valid end-effector poses paramterised with 6 element vectors
    tsr_sampler_ = createIncrementalSampler(tsr_bounds, SamplerType::RANDOM);

    // go through tsr bounds and check were we need to add tolerance
    std::vector<Bounds> tsr_perturbations;
    has_tolerance_.clear();
    for (size_t dim{ 0 }; dim < tsr_bounds.size(); ++dim)
    {
        if (tsr_bounds[dim].lower == tsr_bounds[dim].upper)
        {
            tsr_perturbations.push_back({ 0.0, 0.0 });
            has_tolerance_.push_back(0);
        }
        else
        {
            tsr_perturbations.push_back({ -settings_.D, settings_.D });
            has_tolerance_.push_back(1);
        }
    }
    tsr_local_sampler_ = createIncrementalSampler(tsr_perturbations, SamplerType::RANDOM);
}

Solution OrioloPlanner::solve(const std::vector<ocpl::TSR>& task)
{
    initializeTaskSpaceSamplers(task.at(0).bounds.asVector());
    Solution sol;
    if (settings_.METHOD == "greedy")
    {
        sol.path = greedy(task);
        sol.success = sol.path.size() == task.size();
    }
    else if (settings_.METHOD == "rrtlike")
    {
        sol.path = rrtLike(task);
        sol.success = sol.path.size() == task.size();
    }
    else if (settings_.METHOD == "quispe")
    {
        sol.path = greedy2(task);
        sol.success = sol.path.size() == task.size();
    }
    else
    {
        throw std::runtime_error("Unkown planning method in OrioliPlanner");
    }
    return sol;
}

JointPositions OrioloPlanner::invKin(const TSR& tsr, const JointPositions& q_red)
{
    assert(tsr_sampler_ != nullptr);

    Transform tf = tsr.valuesToPose(tsr_sampler_->getSample());
    IKSolution sol = robot_.ik(tf, q_red);
    if (sol.empty())
    {
        return {};
    }
    else
    {
        // TODO select a random one with proper random number generator?
        return sol[rand() % sol.size()];
    }
}

JointPositions OrioloPlanner::invKin(const TSR& tsr, const JointPositions& q_red, const JointPositions& q_bias)
{
    assert(tsr_local_sampler_ != nullptr);

    std::vector<double> v_prev = tsr.poseToValues(robot_.fk(q_bias));
    std::vector<double> v_bias = tsr_local_sampler_->getSample();

    // assume the variable that changes between two poses has no tolerance
    for (size_t dim{ 0 }; dim < has_tolerance_.size(); ++dim)
    {
        if (has_tolerance_[dim] == 1)
        {
            v_bias.at(dim) += v_prev.at(dim);
        }
    }

    Transform tf = tsr.valuesToPose(v_bias);
    IKSolution sol = robot_.ik(tf, q_red);
    for (auto q_sol : sol)
    {
        if (LInfNormDiff2(q_sol, q_bias) < settings_.D)
        {
            return q_sol;
        }
    }
    return {};
}

/** \brief return random positions for redundant joints. Centered around a bias position.
 *
 * In paper it is described as "generated through a limited random perturbation of q_red_bias".
 * I interpret limited as using the maximum displacement parameter d;
 * **/
JointPositions OrioloPlanner::randRed(const JointPositions& q_bias)
{
    JointPositions q_red_random(robot_.num_red_dof);
    JointPositions perturbation = q_red_local_sampler_->getSample();
    for (size_t i{ 0 }; i < robot_.num_red_dof; ++i)
        q_red_random[i] = clip(q_bias[i] + perturbation[i], robot_.joint_limits[i].lower, robot_.joint_limits[i].upper);
    return q_red_random;
};

JointPositions OrioloPlanner::randRed()
{
    auto q = q_sampler_->getSample();
    JointPositions(q.begin(), q.begin() + robot_.num_red_dof);
    return JointPositions(q.begin(), q.begin() + robot_.num_red_dof);
};

JointPositions OrioloPlanner::randConf()
{
    return q_sampler_->getSample();
}

JointPositions OrioloPlanner::randConf(const TSR& tsr)
{
    auto q_random = randRed();
    return invKin(tsr, q_random);  // this is an emtpy vector if invKin failed
}

JointPositions OrioloPlanner::randConf(const TSR& tsr, const JointPositions& q_bias)
{
    auto q_red = randRed(q_bias);
    auto q_ik = invKin(tsr, q_red, q_bias);
    return q_ik;  // this is an emtpy vector if invKin failed
}

JointPositions OrioloPlanner::sample(const TSR& tsr)
{
    assert(tsr_sampler_ != nullptr);

    // sample c-space for redundant joints
    auto q_red = randRed();

    // sample t-space tolerance
    Transform tf = tsr.valuesToPose(tsr_sampler_->getSample());

    // solve inverse kinematics te calculate base joints
    IKSolution sol = robot_.ik(tf, q_red);

    // now select a random ik_solution if tf was reachable
    if (sol.empty())
    {
        return {};
    }
    else
    {
        // TODO select a random one with proper random number generator?
        return sol[rand() % sol.size()];
    }
}

JointPositions OrioloPlanner::sample(const TSR& tsr, const JointPositions& q_bias)
{
    assert(tsr_local_sampler_ != nullptr);

    // get a biased sample for the redundant joints
    auto q_red = randRed(q_bias);

    // // now comes a trickier part, get a biased sample for the end-effector pose
    std::vector<double> v_prev = tsr.poseToValues(robot_.fk(q_bias));
    std::vector<double> v_bias = tsr_local_sampler_->getSample();

    assert(v_prev.size() == has_tolerance_.size());
    assert(v_bias.size() == has_tolerance_.size());

    // assume the variable that changes between two poses has no tolerance
    for (size_t dim{ 0 }; dim < has_tolerance_.size(); ++dim)
    {
        if (has_tolerance_[dim] == 1)
        {
            v_bias[dim] += v_prev[dim];
        }
    }
    Transform tf = tsr.valuesToPose(v_bias);

    // solve inverse kinematics te calculate base joints
    IKSolution sol = robot_.ik(tf, q_red);
    for (auto q_sol : sol)
    {
        if (LInfNormDiff2(q_sol, q_bias) < settings_.D)
        {
            return q_sol;
        }
    }
    return {};
}

PriorityQueue OrioloPlanner::getLocalSamples(const TSR& tsr, const JointPositions& q_bias, int num_samples)
{
    assert(tsr_local_sampler_ != nullptr);

    // get a biased sample for the redundant joints
    std::vector<JointPositions> q_red_samples;
    auto red_samples = q_red_local_sampler_->getSamples(num_samples);
    for (auto perturbation : red_samples)
    {
        JointPositions q_red_random(robot_.num_red_dof);
        for (size_t i{ 0 }; i < robot_.num_red_dof; ++i)
            q_red_random[i] = clip(q_bias[i] + perturbation[i], robot_.joint_limits[i].lower, robot_.joint_limits[i].upper);
        q_red_samples.push_back(q_red_random);
    }

    std::vector<double> v_prev = tsr.poseToValues(robot_.fk(q_bias));
    std::vector<Transform> tf_samples;
    auto tsr_samples = tsr_local_sampler_->getSamples(num_samples);
    for (auto tsr_sample : tsr_samples)
    {
        // // now comes a trickier part, get a biased sample for the end-effector pose
        std::vector<double> v_bias = tsr_sample;
        // assume the variable that changes between two poses has no tolerance
        for (size_t dim{ 0 }; dim < has_tolerance_.size(); ++dim)
        {
            if (has_tolerance_[dim] == 1)
            {
                v_bias.at(dim) += v_prev.at(dim);
            }
        }
        Transform tf = tsr.valuesToPose(v_bias);
        tf_samples.push_back(tf);
    }

    // compare function
    auto cmp = [q_bias](const JointPositions& left, const JointPositions& right) {
        // std::cout << "cmp: " << q_bias.back() << "\n";
        double d_left = norm2Diff(left, q_bias);
        double d_right = norm2Diff(right, q_bias);
        return d_left > d_right;
    };

    // solve inverse kinematics te calculate base joints
    PriorityQueue samples(cmp);
    for (size_t i{ 0 }; i < tf_samples.size(); ++i)
    {
        IKSolution sol = robot_.ik(tf_samples[i], q_red_samples[i]);
        for (auto q_sol : sol)
        {
            if (LInfNormDiff2(q_sol, q_bias) < settings_.D)
            {
                samples.push(q_sol);
            }
        }
    }
    return samples;
}

JointPositions OrioloPlanner::findChildren(const TSR& tsr, const JointPositions& q_bias, int num_samples)
{
    JointPositions q_child{};
    size_t iters{ 0 };
    bool success{ false };
    while (!success && iters < settings_.MAX_SHOTS)
    {
        q_child = sample(tsr, q_bias);
        if (!q_child.empty())
        {
            success = true;
        }
    }
    return q_child;
}

bool OrioloPlanner::noColl(const JointPositions& q)
{
    return robot_.isValid(q);
}

bool OrioloPlanner::noColl(const JointPositions& q_from, const JointPositions& q_to)
{
    if (!robot_.isValid(q_from))
        return false;
    // TODO figure out what they do mean in the paper
    // now fix the number of steps
    const int steps{ 3 };
    for (int step = 1; step < steps; ++step)
    {
        auto q_step = interpolate(q_from, q_to, static_cast<double>(step) / (steps - 1));
        if (!robot_.isValid(q_step))
        {
            return false;
        }
    }
    return true;
}

std::vector<JointPositions> OrioloPlanner::greedy2(const std::vector<TSR>& task)
{
    initializeTaskSpaceSamplers(task[0].bounds.asVector());
    std::vector<JointPositions> path;
    size_t iters{ 0 };
    bool success{ false };
    JointPositions q_start{}, q_current{};

    // try for different start configs until success
    while (!success && iters < settings_.MAX_ITER)
    {
        std::cout << "Start config iterations " << iters << "\n";
        q_start = sample(task[0]);
        if (!q_start.empty())
        {
            path = { q_start };
            q_current = q_start;

            // depth first with only a single child for every node
            for (size_t i{ 1 }; i < task.size(); ++i)
            {
                size_t iters_2{ 0 };
                bool success_2{ false };
                while (!success_2 && iters_2 < settings_.MAX_SHOTS)
                {
                    std::cout << "Find next config iteration " << iters_2 << "\n";
                    auto local_samples = getLocalSamples(task[i], path.back(), 1000);
                    if (!local_samples.empty())
                    {
                        // find a collision free connection
                        while (!success_2 && !local_samples.empty())
                        {
                            std::cout << "Local samples collision checking left: " << local_samples.size() << "\n";
                            auto q_try = local_samples.top();
                            local_samples.pop();
                            if (noColl(path.back(), q_try))
                            {
                                path.push_back(q_try);
                                std::cout << "cost: " << LInfNormDiff2(path[i], path[i - 1]) << "\n";
                                success_2 = true;
                            }
                        }
                    }
                    iters_2++;
                }
                if (!success_2)
                {
                    break;
                }
            }
            if (path.size() == task.size())
            {
                success = true;
            }
        }
        iters++;
    }
    return path;
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
        while (l < settings_.MAX_SHOTS && !success)
        {
            // q_new = randConf(task.at(j + 1), q_current);
            q_new = sample(task[j + 1], q_current);
            if (!q_new.empty() && noColl(q_current, q_new))
            {
                path.push_back(q_new);
                success = true;
            }
            l++;
        }  // end inner loop

        // give up if this is reached for any path point
        if (l == settings_.MAX_SHOTS)
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
    while (iters < settings_.MAX_ITER && !success)
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

std::pair<bool, graph::NodeData> OrioloPlanner::extend(const std::vector<ocpl::TSR>& task, graph::Tree& tree)
{
    auto q_rand = randConf();
    auto n_near = getNear(q_rand, tree);
    auto q_extend = interpolate(n_near->data.q, q_rand, EXTEND_STEP_);

    // only use redundant joint from extended config
    JointPositions q_ext_red(q_extend.begin(), q_extend.begin() + robot_.num_red_dof);

    // solve inverse kinematics of the next waypoint with the nearest node as bias
    size_t next_waypoint = n_near->data.waypoint + 1;
    auto q_new = invKin(task[next_waypoint], q_ext_red, n_near->data.q);

    if (!q_new.empty() && noColl(n_near->data.q, q_new))
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
    while (current_waypoint != goal_waypoint && iters < settings_.MAX_ITER)
    {
        tree.clear();

        // create a tree from a random start configurations
        auto q0 = randConf(task.at(0));
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
        } while (current_waypoint != goal_waypoint && extend_iters < settings_.MAX_EXTEND);

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

}  // namespace oriolo
}  // namespace ocpl
