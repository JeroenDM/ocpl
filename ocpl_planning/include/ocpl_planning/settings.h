#pragma once

#include <array>
#include <vector>

namespace ocpl
{
/** \brief How do we sample the solutions space?
 *
 * GRID: a grid with a fixed resultion.
 * INCR: incrementally generate samples that try to stay uniform.
 *
 * We can do the latter but using a pseudo-random number generator or
 * a deterministic sequence generator.
 *
 * **/
enum class SampleMethod
{
    GRID,
    INCR_RAN,
    INCR_DET
};

struct PlannerSettings
{
    /** Does the robot have more than 3 / 6 joints for 2D / 3D case. **/
    bool is_redundant{false};

    /* Sample with a grid or incrementally? */
    SampleMethod sample_method;

    // settings for grid samplers

    /** The number of samples for each dimension in a task space regions.
     * (x, y, x, rx, ry, rz)
     * **/
    std::array<int, 6> tsr_resolution;

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
    int max_iters{50};
};

}  // namespace ocpl
