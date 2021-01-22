#pragma once

#include <functional>
#include <vector>

#include <ocpl_tsr/task_space_regions.h>
#include <ocpl_sampling/sampler.h>

namespace oriolo
{
typedef std::vector<double> JointPositions;
typedef std::vector<JointPositions> IKSolution;
typedef std::function<std::vector<JointPositions>(const ocpl::Transform&, const JointPositions&)> IKFunction;

namespace magic
{
// d: "maximum allowed displacement of a single joint."
constexpr double D{ 0.1 };
// upperbound on calls to randConf to find joint positions for a waypoint along thepath
constexpr size_t MAX_SHOTS{ 100 };
// maximum iterations to find a good start configuration for greedy search
constexpr size_t MAX_ITER{ 1000 };
}  // namespace magic

ocpl::SamplerPtr createRedundantSampler(const size_t num_red_joints);

class Planner
{
  private:
    IKFunction ik_fun_;
    ocpl::SamplerPtr red_sampler_;
    ocpl::SamplerPtr q_sampler_;
    ocpl::SamplerPtr tsr_sampler_;
    size_t NUM_RED_DOF_{0};

  public:
    Planner(IKFunction ik_fun, std::vector<ocpl::Bounds>& joint_limits, std::vector<ocpl::Bounds>& tsr_bounds);

    /** \brief Unbiased inverse kinematics for random samle in task space regions. **/
    JointPositions invKin(const ocpl::TSR& tsr, const JointPositions& q_red);

    /** \brief Inverse kinematics with a bias value to stay close to.
     *
     * Returns empty vector if it failed.
     * **/
    JointPositions invKin(const ocpl::TSR& tsr, const JointPositions& q_red, const JointPositions& q_bias);

    /** \brief return random positions for redundant joints. Centered around a bias position.
     *
     * In paper it is described as "generated through a limited random perturbation of q_red_bias".
     * I interpret limited as using the maximum displacement parameter d;
     * **/
    JointPositions randRed(const JointPositions& q_bias);

    // all variations on randConf to get random joint positions
    JointPositions randConf();
    JointPositions randConf(const ocpl::TSR& tsr);
    JointPositions randConf(const ocpl::TSR& tsr, const JointPositions& q_bias);
};

}  // namespace oriolo
