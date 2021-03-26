#include "ocpl_benchmark/benchmark.h"
#include "ocpl_benchmark/timer.h"
#include "ocpl_benchmark/data_logger.h"

#include <vector>
#include <iostream>

#include <ocpl_tsr/task_space_regions.h>
#include <ocpl_planning/types.h>
#include <ocpl_planning/acro_planner.h>

namespace ocpl
{
void runBenchmark(const std::string& name, const Robot& robot, const std::vector<TSR>& task, Planner& planner,
                  const std::vector<PlannerSettings>& settings, int num_repeats)
{
    std::cout << "Starting benchmark named: " << name << " repeating it " << num_repeats << " times \n";
    Logger log(name);
    log.header("settings,run,success,time,cost");

    for (auto ps : settings)
    {
        std::cout << "--- planner " << ps.name << " ---\n";
        planner.changeSettings(ps);

        for (int run{ 0 }; run < num_repeats; ++run)
        {
            std::cout << "--- run " << run << " ---\n";

            Timer timer;
            timer.start();
            Solution solution = planner.solve(task);
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
void runBenchmark(const std::string& name, const Robot& robot, const std::vector<std::vector<TSR>>& tasks,
                  Planner& planner, const std::vector<PlannerSettings>& settings, int num_repeats)
{
    std::cout << "Starting benchmark named: " << name << " repeating it " << num_repeats << " times \n";
    Logger log(name);
    log.header("settings,run,task,success,time,cost");

    for (auto ps : settings)
    {
        std::cout << "--- planner " << ps.name << " ---\n";
        planner.changeSettings(ps);

        for (std::size_t task_index{ 0 }; task_index < tasks.size(); ++task_index)
        {
            for (int run{ 0 }; run < num_repeats; ++run)
            {
                std::cout << "--- task " << task_index << ", run " << run << " ---\n";

                Timer timer;
                timer.start();
                Solution solution = planner.solve(tasks[task_index]);
                double t = timer.stop();

                log.log(ps.name);
                log.log(run);
                log.log(task_index);
                log.log(solution.success);
                log.log(t);
                log.log(solution.cost);
                log.nextLine();
            }
        }
    }
}

}  // namespace ocpl
