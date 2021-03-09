#include <ocpl_planning/planner_base.h>

#include <ocpl_planning/factories.h>

namespace ocpl
{
/************************************************************************
 * SAMPLER INITIALIZATION
 * **********************************************************************/
Planner::Planner(const Robot& robot, const PlannerSettings& settings) : robot_(robot), settings_(settings)
{
    // sampler to generate perturbations on redundant joints with max deviation d around zero
    q_red_local_sampler_ = createLocalSampler(robot.num_red_dof, settings_.cspace_delta, settings_.sampler_type,
                                              settings_.redundant_joints_resolution);

    // sampler to generate completely random robot configurations inside the limits
    // at the moment this sampler is never used in the case we do grid sampling
    // but it might be usefull in some cases for RRT like planners that use a grid??
    // therefore extend the resolution of the last redundant joint to all the other joints.
    std::vector<int> all_joints_resolution;
    if (!settings_.redundant_joints_resolution.empty())
    {
        all_joints_resolution = settings_.redundant_joints_resolution;
        all_joints_resolution.resize(robot.num_dof);
        std::fill(all_joints_resolution.begin() + robot.num_red_dof, all_joints_resolution.end(),
                  settings_.redundant_joints_resolution.back());
    }
    q_sampler_ = createSampler(robot.joint_limits, settings_.sampler_type, all_joints_resolution);
}

void Planner::initializeTaskSpaceSamplers(const std::vector<Bounds> tsr_bounds)
{
    //  sampler to generate random valid end-effector poses paramterised with 6 element vectors
    tsr_sampler_ = createSampler(tsr_bounds, settings_.sampler_type, settings_.tsr_resolution);

    // go through tsr bounds and check were we need to add tolerance
    std::vector<Bounds> tsr_perturbations;
    for (size_t dim{ 0 }; dim < tsr_bounds.size(); ++dim)
    {
        if (tsr_bounds[dim].lower == tsr_bounds[dim].upper)
        {
            tsr_perturbations.push_back({ 0.0, 0.0 });
        }
        else
        {
            tsr_perturbations.push_back({ -settings_.tspace_delta, settings_.tspace_delta });
        }
    }
    tsr_local_sampler_ = createSampler(tsr_perturbations, settings_.sampler_type, settings_.tsr_resolution);
}

}  // namespace ocpl
