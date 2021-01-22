#include <ocpl_planning/oriolo.h>

#include <stdlib.h>

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

Planner::Planner(IKFunction ik_fun, std::vector<Bounds>& joint_limits, std::vector<Bounds>& tsr_bounds)
  : ik_fun_(ik_fun)
{
    // sampler to generate perturbations on redundant joints with max deviation d
    red_sampler_ = oriolo::createRedundantSampler(3);
    // sampler to generate completely random robot configurations inside the limits
    q_sampler_ = createIncrementalSampler(joint_limits, SamplerType::RANDOM);
    // sampler to generate random valid end-effector poses paramterised with 6 element vectors
    tsr_sampler_ = createIncrementalSampler(tsr_bounds, SamplerType::RANDOM);
    NUM_RED_DOF_ = joint_limits.size() - 3;
}

JointPositions Planner::invKin(const TSR& tsr, const JointPositions& q_red)
{
    Transform tf = tsr.valuesToPose(tsr_sampler_->getSamples(1)[0]);
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
    Transform tf = tsr.valuesToPose(tsr_sampler_->getSamples(1)[0]);
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
    JointPositions perturbation = red_sampler_->getSamples(1)[0];
    for (size_t i{ 0 }; i < NUM_RED_DOF_; ++i)
        q_red_random[i] = q_bias[i] + perturbation[i];
    return q_red_random;
};

JointPositions Planner::randConf()
{
    return q_sampler_->getSamples(1)[0];
}

JointPositions Planner::randConf(const TSR& tsr)
{
    auto q_random = randConf();
    return invKin(tsr, q_random);  // this is an emtpy vector if invKin failed
}

JointPositions Planner::randConf(const TSR& tsr, const JointPositions& q_bias)
{
    auto q_red = randRed(q_bias);
    auto q_ik = invKin(tsr, q_red, q_bias);
    return q_ik;  // this is an emtpy vector if invKin failed
}
}  // namespace oriolo
