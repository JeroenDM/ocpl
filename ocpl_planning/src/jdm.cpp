#include <ocpl_planning/jdm.h>

#include <ocpl_planning/cost_functions.h>
#include <ocpl_planning/planners.h>

namespace ocpl
{
namespace jdm
{
JdmPlanner::JdmPlanner(const std::string& name, const Robot& robot, const PlannerSettings& settings)
  : Planner(name, robot), settings_(settings)
{
}

Solution JdmPlanner::solve(const std::vector<TSR>& task)
{
    std::vector<Bounds> red_joint_limits(robot_.joint_limits.begin(), robot_.joint_limits.begin() + robot_.num_red_dof);
    return ocpl::solve(task, red_joint_limits, robot_.ik, robot_.isValid, L2NormDiff2, zeroStateCost, settings_);
}

}  // namespace jdm
}  // namespace ocpl
