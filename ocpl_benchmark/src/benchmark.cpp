#include "ocpl_benchmark/benchmark.h"
#include "ocpl_benchmark/timer.h"
#include "ocpl_benchmark/data_logger.h"

#include <vector>
#include <iostream>

#include <ocpl_tsr/task_space_regions.h>
#include <ocpl_planning/settings.h>
#include <ocpl_planning/planners.h>

namespace ocpl
{
void runBenchmark(const std::string& name, const Problem& problem, const std::vector<TSR>& task,
                  const std::vector<PlannerSettings>& settings, int num_repeats)
{
    std::cout << "Starting benchmark named: " << name << " repeating it " << num_repeats << " times \n";
    Logger log(name);
    log.header("settings,run,success,time,cost");

    for (auto ps : settings)
    {
        std::cout << "--- planner " << ps.name << " ---\n";

        for (int run{ 0 }; run < num_repeats; ++run)
        {
            std::cout << "--- run " << run << " ---\n";

            Timer timer;
            timer.start();
            Solution solution = solve(task, problem.redundant_joint_limits, problem.ik_fun, problem.is_valid_fun,
                                      problem.path_cost_fun, problem.state_cost_fun, ps);
            double t = timer.stop();

            log.log(ps.name);
            log.log(run);
            log.log(solution.success);
            log.log(t);
            log.log(solution.cost);
            log.nextLine();
        }
    }
}

}  // namespace ocpl
