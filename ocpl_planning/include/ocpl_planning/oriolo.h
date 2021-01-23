#pragma once

#include <functional>
#include <vector>

#include <ocpl_tsr/task_space_regions.h>
#include <ocpl_sampling/sampler.h>

namespace oriolo
{
typedef std::vector<double> JointPositions;
typedef std::vector<JointPositions> IKSolution;
typedef std::function<ocpl::Transform(const JointPositions&)> FKFun;
typedef std::function<std::vector<JointPositions>(const ocpl::Transform&, const JointPositions&)> IKFun;
typedef std::function<bool(const JointPositions&)> IsValidFun;

namespace magic
{
// d: "maximum allowed displacement of a single joint."
constexpr double D{ 0.2 };
// upperbound on calls to randConf to find joint positions for a waypoint along thepath
constexpr size_t MAX_SHOTS{ 100 };
// maximum iterations to find a good start configuration for greedy search
constexpr size_t MAX_ITER{ 1000 };
}  // namespace magic

ocpl::SamplerPtr createRedundantSampler(const size_t num_red_joints);

/** \brief Simple interpolation between two vectors. Returns one vector.**/
template <typename T>
inline std::vector<T> interpolate(std::vector<T> q_from, std::vector<T> q_to, T s)
{
    std::vector<T> q(q_from.size());
    for (std::size_t i = 0; i < q_from.size(); ++i)
    {
        q[i] = (1 - s) * q_from[i] + s * q_to[i];
    }
    return q;
}

class Planner
{
  private:
    FKFun fk_fun_;
    IKFun ik_fun_;
    IsValidFun is_valid_;

    ocpl::SamplerPtr red_sampler_;
    ocpl::SamplerPtr q_sampler_;
    ocpl::SamplerPtr tsr_sampler_;
    ocpl::SamplerPtr tsr_sampler_small_;
    size_t NUM_RED_DOF_{ 0 };
    std::vector<int> has_tolerance_;

  public:
    Planner(FKFun fk_fun, IKFun ik_fun, IsValidFun is_valid, std::vector<ocpl::Bounds>& joint_limits,
            std::vector<ocpl::Bounds>& tsr_bounds, size_t num_red_dof);

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
    JointPositions randRed();

    // all variations on randConf to get random joint positions
    JointPositions randConf();
    JointPositions randConf(const ocpl::TSR& tsr);
    JointPositions randConf(const ocpl::TSR& tsr, const JointPositions& q_bias);

    // state and path validation (collision checking)
    bool noColl(const JointPositions& q);
    bool noColl(const JointPositions& q_from, const JointPositions& q_to);

    std::vector<JointPositions> step(size_t start_index, size_t stop_index, const JointPositions& q_start,
                                     const std::vector<ocpl::TSR>& task);
    std::vector<JointPositions> greedy(const std::vector<ocpl::TSR>& task);
};

}  // namespace oriolo
