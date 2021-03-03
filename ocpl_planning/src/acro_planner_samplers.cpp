#include <ocpl_planning/acro_planner.h>

#include <ocpl_planning/thread_save_logging.h>

namespace ocpl
{
/************************************************************************
 * GLOBAL SAMPLING
 * **********************************************************************/
std::vector<std::vector<JointPositions>>
UnifiedPlanner::sampleGlobalIncremental(std::vector<std::function<IKSolution()>> path_samplers)
{
    std::vector<std::vector<JointPositions>> graph_data;
    graph_data.resize(path_samplers.size());

    TSLogger logger;  // Thread save logging

#pragma omp parallel
#pragma omp for
    for (std::size_t i = 0; i < path_samplers.size(); ++i)
    {
        int iters{ 0 };
        std::vector<JointPositions> valid_samples;
        while (iters < settings_.max_iters && valid_samples.size() < settings_.min_valid_samples)
        {
            auto s = path_samplers[i]();

            // add the new joint positions to valid_samples (stl can be ugly...)
            valid_samples.reserve(valid_samples.size() + std::distance(s.begin(), s.end()));
            valid_samples.insert(valid_samples.end(), s.begin(), s.end());

            iters++;
        }
        graph_data[i] = valid_samples;

        logger.logWaypoint(i, valid_samples.size());

        if (iters == settings_.max_iters)
            logger.log("ocpl_planner: maximum number of iterations reached.\n");
    }
    return graph_data;
}

std::vector<std::vector<JointPositions>>
UnifiedPlanner::sampleGlobalGrid(std::vector<std::function<IKSolution()>> path_samplers)
{
    std::vector<std::vector<JointPositions>> graph_data;
    graph_data.resize(path_samplers.size());

    TSLogger logger;  // Thread save logging

#pragma omp parallel
#pragma omp for
    for (std::size_t i = 0; i < path_samplers.size(); ++i)
    {
        graph_data[i] = path_samplers[i]();

        logger.logWaypoint(i, graph_data[i].size());
    }
    return graph_data;
}

/************************************************************************
 * LOCAL SAMPLING
 * **********************************************************************/
std::vector<JointPositions> UnifiedPlanner::sampleLocalIncremental(
    const JointPositions& q_bias, std::function<IKSolution(const JointPositions&)> local_sampler)
{
    int iters{ 0 };
    std::vector<JointPositions> valid_samples;
    while (iters < settings_.max_iters && valid_samples.size() < settings_.min_valid_samples)
    {
        auto s = local_sampler(q_bias);

        // add the new joint positions to valid_samples (stl can be ugly...)
        valid_samples.reserve(valid_samples.size() + std::distance(s.begin(), s.end()));
        valid_samples.insert(valid_samples.end(), s.begin(), s.end());

        iters++;
    }

    return valid_samples;
}
}  // namespace ocpl
