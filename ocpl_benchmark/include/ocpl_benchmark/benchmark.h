#pragma once

#include <vector>

#include <ocpl_tsr/task_space_regions.h>
#include <ocpl_planning/types.h>
#include <ocpl_planning/planner_base.h>
#include <ocpl_planning/oriolo.h>

namespace ocpl
{
/** \brief Solve a single planning problem with a range of planner settings.
 * Optionally we can repeat it multiple times (for probabilist planning and timing results).
 * **/
void runBenchmark(const std::string& name, const Robot& robot, const std::vector<TSR>& task, Planner& planner,
                  const std::vector<PlannerSettings>& settings, int num_repeats);
}  // namespace ocpl
