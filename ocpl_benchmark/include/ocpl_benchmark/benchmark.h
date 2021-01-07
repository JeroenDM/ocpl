#pragma once

#include <vector>

#include <ocpl_tsr/task_space_regions.h>
#include <ocpl_planning/settings.h>
#include <ocpl_planning/planners.h>

namespace ocpl
{
/** \brief Solve a single planning problem with a range of planner settings.
 * Optionally we can repeat it multiple times (for probabilist planning and timing results).
 * **/
void runBenchmark(const Problem& problem, const std::vector<TSR>& task, const std::vector<PlannerSettings>& settings,
                  int num_repeats = 1);
}  // namespace ocpl
