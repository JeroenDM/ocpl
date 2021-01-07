#include "ocpl_benchmark/benchmark.h"
#include "ocpl_benchmark/timer.h"

#include <vector>
#include <iostream>

#include <ocpl_tsr/task_space_regions.h>
#include <ocpl_planning/settings.h>
#include <ocpl_planning/planners.h>

namespace ocpl
{
void runBenchmark(const Problem& problem, const std::vector<TSR>& task, const std::vector<PlannerSettings>& settings,
                  int num_repeats)
{
    std::cout << "success,\ttime,\tcost\n";
    for (int run{ 0 }; run < num_repeats; ++run)
    {
        std::cout << "run " << run << " ---\n";
        for (auto ps : settings)
        {
            Timer timer;
            timer.start();
            Solution solution = solve(task, problem.redundant_joint_limits, problem.ik_fun, problem.is_valid_fun,
                                      problem.path_cost_fun, problem.state_cost_fun, ps);
            double t = timer.stop();
            std::cout << solution.success << ",\t" << t << ",\t" << solution.cost << "\n";
        }
    }
}

}  // namespace ocpl
