#pragma once

#include <ocpl_planning/types.h>
#include <ocpl_sampling/sampler.h>

namespace ocpl
{
struct SampleSpaceSettings
{
    /** Does the robot have more than 3 / 6 joints for 2D / 3D case. **/
    bool is_redundant{ false };

    /* Sample with a grid or incrementally? */
    SamplerType sampler_type;

    /** How many collision free samples do we want at least for every path point?
     *
     * This defaults to 1, to make sure the sample loop at least runs 1 time,
     * when the maximum number of iterations is set to 1.
     * **/
    int min_valid_samples{ 1 };

    double cspace_delta{ 0.1 };
    double tspace_delta{ 0.1 };

    /** The number of samples for each dimension in a task space regions.
     * (x, y, x, rx, ry, rz)
     * **/
    std::vector<int> tsr_resolution{};

    /** The number of samples for all the redundant joints. **/
    std::vector<int> redundant_joints_resolution{};
};

class SampleSpace
{
    Robot robot_;
    SampleSpaceSettings settings_;

    SamplerPtr q_red_local_sampler_;
    SamplerPtr q_sampler_;
    SamplerPtr tsr_sampler_;
    SamplerPtr tsr_local_sampler_;

    int debug_;

  public:
    SampleSpace(const Robot& robot, const SampleSpaceSettings& settings, bool debug = 0);

    /** Setup sampler in joint space. **/
    void initializeJointSpaceSamplers();

    /** Use different settings.
     * This reinitializes the joint space samplers.
     * */
    virtual void changeSettings(const SampleSpaceSettings& new_settings);

    /** Setup samplers in task space for a specific task. **/
    void initializeTaskSpaceSamplers(const std::vector<Bounds> tsr_bounds);

    /** \brief Sample only the redundant joints. **/
    JointPositions sampleRedJoints();

    /** \brief Sampled redundant joints around a given bias configuration.
     *
     * The interval with is [-d, d] where d is the `settings_.cspace_delta` parameter.
     * It also keeps the samle within the joint limits.
     * **/
    JointPositions sampleRedJoints(const JointPositions& q_bias);

    /** \brief Get a (random) robot configurations for a given waypoint along the path. **/
    JointPositions sample(const TSR& tsr);

    /** \brief Get a (random) biased robot configurations for a given waypoint along the path.
     *
     * The solution space for the given waypoint is sampled in a regions around the q_bias;
     *
     * **/
    JointPositions sample(const TSR& tsr, const JointPositions& q_bias);
};
}  // namespace ocpl
