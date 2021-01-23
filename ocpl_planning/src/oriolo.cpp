#include <ocpl_planning/oriolo.h>

#include <stdlib.h>
#include <cmath>

#include <ocpl_planning/cost_functions.h>
#include <ocpl_sampling/random_sampler.h>
#include <ocpl_planning/factories.h>

namespace oriolo
{
using namespace ocpl;

SamplerPtr createRedundantSampler(const size_t num_red_joints)
{
    auto sampler = std::make_shared<RandomSampler>();
    for (size_t i{ 0 }; i < num_red_joints; ++i)
    {
        sampler->addDimension(-magic::D, magic::D);
    }
    return sampler;
}

Planner::Planner(FKFun fk_fun, IKFun ik_fun, IsValidFun is_valid, std::vector<Bounds>& joint_limits,
                 std::vector<Bounds>& tsr_bounds, size_t num_red_dof)
  : fk_fun_(fk_fun), ik_fun_(ik_fun), is_valid_(is_valid), NUM_RED_DOF_(num_red_dof)
{
    // sampler to generate perturbations on redundant joints with max deviation d
    red_sampler_ = oriolo::createRedundantSampler(3);
    // sampler to generate completely random robot configurations inside the limits
    q_sampler_ = createIncrementalSampler(joint_limits, SamplerType::RANDOM);
    // sampler to generate random valid end-effector poses paramterised with 6 element vectors
    tsr_sampler_ = createIncrementalSampler(tsr_bounds, SamplerType::RANDOM);

    // go through tsr bounds and check were we need to add tolerance
    std::vector<Bounds> tsr_perturbations;
    for (size_t dim{ 0 }; dim < tsr_bounds.size(); ++dim)
    {
        if (tsr_bounds[dim].lower == tsr_bounds[dim].upper)
        {
            tsr_perturbations.push_back({ 0.0, 0.0 });
            has_tolerance_.push_back(0);
        }
        else
        {
            tsr_perturbations.push_back({ -magic::D, magic::D });
            has_tolerance_.push_back(1);
        }
    }
    tsr_sampler_small_ = createIncrementalSampler(tsr_perturbations, SamplerType::RANDOM);
}

JointPositions Planner::invKin(const TSR& tsr, const JointPositions& q_red)
{
    Transform tf = tsr.valuesToPose(tsr_sampler_->getSample());
    IKSolution sol = ik_fun_(tf, q_red);
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

JointPositions Planner::invKin(const TSR& tsr, const JointPositions& q_red, const JointPositions& q_bias)
{
    std::vector<double> v_prev = tsr.poseToValues(fk_fun_(q_bias));
    std::vector<double> v_bias = tsr_sampler_small_->getSample();

    // assume the variable that changes between two poses has no tolerance
    for (size_t dim{ 0 }; dim < has_tolerance_.size(); ++dim)
    {
        if (has_tolerance_[dim] == 1)
        {
            v_bias.at(dim) += v_prev.at(dim);
        }
    }

    Transform tf = tsr.valuesToPose(v_bias);
    IKSolution sol = ik_fun_(tf, q_red);
    for (auto q_sol : sol)
    {
        if (LInfNormDiff2(q_sol, q_bias) < magic::D)
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
JointPositions Planner::randRed(const JointPositions& q_bias)
{
    JointPositions q_red_random(NUM_RED_DOF_);
    JointPositions perturbation = red_sampler_->getSample();
    for (size_t i{ 0 }; i < NUM_RED_DOF_; ++i)
        q_red_random[i] = q_bias[i] + perturbation[i];
    return q_red_random;
};

JointPositions Planner::randRed()
{
    auto q = q_sampler_->getSample();
    JointPositions(q.begin(), q.begin() + NUM_RED_DOF_);
    return JointPositions(q.begin(), q.begin() + NUM_RED_DOF_);
};

JointPositions Planner::randConf()
{
    return q_sampler_->getSample();
}

JointPositions Planner::randConf(const TSR& tsr)
{
    auto q_random = randRed();
    return invKin(tsr, q_random);  // this is an emtpy vector if invKin failed
}

JointPositions Planner::randConf(const TSR& tsr, const JointPositions& q_bias)
{
    auto q_red = randRed(q_bias);
    auto q_ik = invKin(tsr, q_red, q_bias);
    return q_ik;  // this is an emtpy vector if invKin failed
}

bool Planner::noColl(const JointPositions& q)
{
    return is_valid_(q);
}

bool Planner::noColl(const JointPositions& q_from, const JointPositions& q_to)
{
    if (!is_valid_(q_from))
        return false;
    // TODO figure out what they do mean in the paper
    // now fix the number of steps
    const int steps{ 3 };
    for (int step = 1; step < steps; ++step)
    {
        auto q_step = interpolate(q_from, q_to, static_cast<double>(step) / (steps - 1));
        if (!is_valid_(q_step))
        {
            return false;
        }
    }
    return true;
}

std::vector<JointPositions> Planner::step(size_t start_index, size_t stop_index, const JointPositions& q_start,
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
        while (l < magic::MAX_SHOTS && !success)
        {
            q_new = randConf(task.at(j + 1), q_current);
            if (!q_new.empty() && noColl(q_current, q_new))
            {
                path.push_back(q_new);
                success = true;
            }
            l++;
        }  // end inner loop

        // give up if this is reached for any path point
        if (l == magic::MAX_SHOTS)
        {
            return {};
        }
        else
        {
            q_current = q_new;
        }
    }  // end outer loop
    return path;
}

std::vector<JointPositions> Planner::greedy(const std::vector<TSR>& task)
{
    std::cout << "Started Orioli greedy planner\n";
    std::vector<JointPositions> path;
    size_t iters{ 0 };
    bool success{ false };
    while (iters < magic::MAX_ITER && !success)
    {
        JointPositions q_start = randConf(task[0]);
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

}  // namespace oriolo
