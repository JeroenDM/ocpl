#pragma once

#include <functional>

#include <ocpl_planning/types.h>
#include <ocpl_sampling/sampler.h>

namespace ocpl
{
struct PlannerSettings
{
    /** A name that is used when writing benchmarking results to a file. **/
    std::string name;

    /** Does the robot have more than 3 / 6 joints for 2D / 3D case. **/
    bool is_redundant{ false };

    /* Sample with a grid or incrementally? */
    SamplerType sampler_type;

    // settings for grid samplers

    /** The number of samples for each dimension in a task space regions.
     * (x, y, x, rx, ry, rz)
     * **/
    std::vector<int> tsr_resolution{};

    /** The number of samples for all the redundant joints. **/
    std::vector<int> redundant_joints_resolution{};

    // settings for incremental samplers

    /** How many samples do we draw from the TSR in every iteration? **/
    int t_space_batch_size{ 0 };

    /** How many samples for the redundant joints do we draw every iteration? **/
    int c_space_batch_size{ 0 };

    /** How many collision free samples do we want at least for every path point?
     *
     * This defaults to 1, to make sure the sample loop at least runs 1 time,
     * when the maximum number of iterations is set to 1.
     * **/
    int min_valid_samples{ 1 };

    /** How many iterations can we try to find valid samples before giving up. **/
    int max_iters{ 50 };

    double cspace_delta{ 0.1 };
    double tspace_delta{ 0.1 };

    /** The state cost is multiplied with this weight before adding it to the path cost. **/
    double state_cost_weight{ 1.0 };

    PlannerType type{ PlannerType::GLOBAL };

    int debug{ 0 }; /** Simple debug flag for logging statements. **/
};

class Planner
{
  protected:
    const std::string name_;
    Robot robot_;
    PlannerSettings settings_;

    SamplerPtr q_red_local_sampler_;
    SamplerPtr q_sampler_;
    SamplerPtr tsr_sampler_;
    SamplerPtr tsr_local_sampler_;

    int debug_{ 0 };

    void initializeTaskSpaceSamplers(const std::vector<Bounds> tsr_bounds);

    /******************************
     * BUILDING BLOCKS
     * ****************************/
    /** \brief Sample only the redundant joints. **/
    JointPositions sampleRedJoints();

    /** \brief Sampled redudant joints around a given bias configuration.
     *
     * The interval with is [-d, d] where d is the `settings_.cspace_delta` parameter.
     * It also keeps the samle within the joint limits.
     * **/
    JointPositions sampleRedJoints(const JointPositions& q_bias);

    /** \brief Solve inverse kinematics and only return solutions close to a bias config.
     *
     * Meaning non of the joints deviate more than `settings_.cspace_delta`.
     * **/
    JointPositions biasedIK(const Transform& tf, const JointPositions& q_red, const JointPositions& q_bias);

    /** \brief Collision checking along linear interpolated path between 2 configs. **/
    bool isPathValid(const JointPositions& q_from, const JointPositions& q_to);

    /** \brief Get a (random) robot configurations for a given waypoint along the path. **/
    JointPositions sample(const TSR& tsr);

    /** \brief Get a (random) biased robot configurations for a given waypoint along the path.
     *
     * The solution space for the given waypoint is sampled in a regions around the q_bias;
     *
     * **/
    JointPositions sample(const TSR& tsr, const JointPositions& q_bias);

  public:
    Planner(const Robot& robot, const PlannerSettings& settings);

    virtual void changeSettings(const PlannerSettings& new_settings)
    {
        settings_ = new_settings;
    }

    virtual Solution solve(const std::vector<TSR>& task) = 0;

    virtual Solution solve(const std::vector<TSR>& task,
                           std::function<double(const JointPositions&, const JointPositions&)> path_cost_fun,
                           std::function<double(const TSR&, const JointPositions&)> state_cost_fun) = 0;

    void setDebugFlag(bool value)
    {
        debug_ = value;
    }
};
}  // namespace ocpl
