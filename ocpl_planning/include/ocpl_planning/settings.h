#pragma once

#include <array>
#include <vector>

#include <ocpl_sampling/sampler.h>

namespace ocpl
{
struct PlannerSettings
{
    /** Does the robot have more than 3 / 6 joints for 2D / 3D case. **/
    bool is_redundant{ false };

    /* Sample with a grid or incrementally? */
    SamplerType sampler_type;

    // settings for grid samplers

    /** The number of samples for each dimension in a task space regions.
     * (x, y, x, rx, ry, rz)
     * **/
    std::vector<int> tsr_resolution;

    /** The number of samples for all the redundant joints. **/
    std::vector<int> redundant_joints_resolution;

    // settings for incremental samplers

    /** How many samples do we draw from the TSR in every iteration? **/
    int t_space_batch_size;

    /** How many samples for the redundant joints do we draw every iteration? **/
    int c_space_batch_size;

    /** How many collision free samples do we want at least for every path point? **/
    int min_valid_samples;

    /** How many iterations can we try to find valid samples before giving up. **/
    int max_iters{ 50 };
};

}  // namespace ocpl
