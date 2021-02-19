#pragma once

#include <vector>

#include <ocpl_tsr/task_space_regions.h>
#include <ocpl_planning/types.h>
#include <ocpl_planning/planners.h>
#include <ocpl_planning/oriolo.h>
#include <ocpl_planning/planner2.h>

namespace ocpl
{
/** \brief Solve a single planning problem with a range of planner settings.
 * Optionally we can repeat it multiple times (for probabilist planning and timing results).
 * **/
void runBenchmark(const std::string& name, const Problem& problem, const std::vector<TSR>& task,
                  const std::vector<PlannerSettings>& settings, int num_repeats);

void runBenchmark(const std::string& name, const Robot& robot, const std::vector<TSR>& task,
                  const std::vector<oriolo::OrioloSpecificSettings>& settings, int num_repeats);

void runBenchmark(const std::string& name, const Robot& robot, const std::vector<TSR>& task,
                  const std::vector<Planner2Settings>& settings, int num_repeats);
}  // namespace ocpl
