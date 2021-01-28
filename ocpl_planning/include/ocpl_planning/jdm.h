#pragma once

#include <ocpl_planning/planners.h>

namespace ocpl
{
namespace jdm
{
// struct JdmSpecificSettings
// {
//     /** Does the robot have more than 3 / 6 joints for 2D / 3D case. **/
//     bool is_redundant{ false };

//     /* Sample with a grid or incrementally? */
//     SamplerType sampler_type;

//     // settings for grid samplers

//     /** The number of samples for each dimension in a task space regions.
//      * (x, y, x, rx, ry, rz)
//      * **/
//     std::vector<int> tsr_resolution{};

//     /** The number of samples for all the redundant joints. **/
//     std::vector<int> redundant_joints_resolution{};

//     // settings for incremental samplers

//     /** How many samples do we draw from the TSR in every iteration? **/
//     int t_space_batch_size{ 0 };

//     /** How many samples for the redundant joints do we draw every iteration? **/
//     int c_space_batch_size{ 0 };

//     /** How many collision free samples do we want at least for every path point?
//      *
//      * This defaults to 1, to make sure the sample loop at least runs 1 time,
//      * when the maximum number of iterations is set to 1.
//      * **/
//     int min_valid_samples{ 1 };

//     /** How many iterations can we try to find valid samples before giving up. **/
//     int max_iters{ 50 };
// };

class JdmPlanner : public Planner
{
    PlannerSettings settings_;

  public:
    JdmPlanner(const std::string& name, const Robot& robot, const PlannerSettings& settings);

    Solution solve(const std::vector<TSR>& task) override;
};

}  // namespace jdm
}  // namespace ocpl
