#include <ocpl_planning/planners.h>

#include <ocpl_graph/tree.h>
#include <ocpl_planning/cost_functions.h>
#include <ocpl_planning/factories.h>
#include <ocpl_planning/thread_save_logging.h>

#include <algorithm>
#include <iostream>
#include <chrono>

namespace ocpl
{
Planner::Planner(const std::string& name, const Robot& robot) : name_(name), robot_(robot)
{
}

std::vector<std::vector<JointPositions>>
createCSpaceGraphIncrementally(std::vector<std::function<IKSolution()>> path_samplers, const PlannerSettings& settings)
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
        while (iters < settings.max_iters && valid_samples.size() < settings.min_valid_samples)
        {
            auto s = path_samplers[i]();

            // add the new joint positions to valid_samples (stl can be ugly...)
            valid_samples.reserve(valid_samples.size() + std::distance(s.begin(), s.end()));
            valid_samples.insert(valid_samples.end(), s.begin(), s.end());

            iters++;
        }
        graph_data[i] = valid_samples;

        logger.logWaypoint(i, valid_samples.size());

        if (iters == settings.max_iters)
            logger.log("ocpl_planner: maximum number of iterations reached.\n");
    }
    return graph_data;
}

std::vector<std::vector<JointPositions>> createCSpaceGraphGrid(std::vector<std::function<IKSolution()>> path_samplers)
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

Solution solve(const std::vector<TSR>& task_space_regions, const JointLimits& redundant_joint_limits,
               std::function<IKSolution(const Transform&, const JointPositions&)> ik_fun,
               std::function<bool(const JointPositions&)> is_valid_fun,
               std::function<double(const JointPositions&, const JointPositions&)> path_cost_fun,
               std::function<double(const TSR&, const JointPositions&)> state_cost_fun, PlannerSettings settings)
{
    // create the path samplers
    std::vector<std::function<IKSolution()>> path_samplers;

    if (settings.is_redundant && redundant_joint_limits.empty())
        std::cout << "The robot is marked redundant, but the joint limits for the redundant joints are not "
                     "specified...\n";

    if (!settings.is_redundant && redundant_joint_limits.size() > 0)
        std::cout << "The robot is not marked redundant, but joint limits for redundant joints where given.\n";

    if (settings.is_redundant)
    {
        for (auto tsr : task_space_regions)
        {
            SamplerPtr t_sampler = createSampler(tsr.bounds.asVector(), settings.sampler_type, settings.tsr_resolution);
            SamplerPtr c_sampler =
                createSampler(redundant_joint_limits, settings.sampler_type, settings.redundant_joints_resolution);
            path_samplers.push_back(
                [tsr, ik_fun, is_valid_fun, redundant_joint_limits, settings, t_sampler, c_sampler]() {
                    IKSolution result;
                    for (auto tsr_values : t_sampler->getSamples(settings.t_space_batch_size))
                    {
                        for (auto q_red : c_sampler->getSamples(settings.c_space_batch_size))
                        {
                            for (auto q : ik_fun(tsr.valuesToPose(tsr_values), q_red))
                            {
                                if (is_valid_fun(q))
                                    result.push_back(q);
                            }
                        }
                    }

                    return result;
                });
        }
    }
    else
    {
        for (const TSR& tsr : task_space_regions)
        {
            SamplerPtr sampler = createSampler(tsr.bounds.asVector(), settings.sampler_type, settings.tsr_resolution);
            path_samplers.push_back([tsr, ik_fun, is_valid_fun, &settings, sampler]() {
                IKSolution result;
                for (auto tsr_values : sampler->getSamples(settings.t_space_batch_size))
                {
                    for (auto q : ik_fun(tsr.valuesToPose(tsr_values), {}))
                    {
                        if (is_valid_fun(q))
                            result.push_back(q);
                    }
                }
                return result;
            });
        }
    }

    // sample valid joint positions along the path
    std::vector<std::vector<JointPositions>> graph_data;
    if (settings.sampler_type == SamplerType::GRID)
    {
        graph_data = createCSpaceGraphGrid(path_samplers);
    }
    else
    {
        auto start = std::chrono::steady_clock::now();
        graph_data = createCSpaceGraphIncrementally(path_samplers, settings);
        auto stop = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = stop - start;
        std::cout << "Sampling time: " << elapsed_seconds.count() << "\n";
    }

    // Convert the end-effector poses to Nodes, so we can search for the shortest path
    // can also be done in parallel if state cost function is heavy
    std::vector<std::vector<NodePtr>> nodes;
    nodes.resize(path_samplers.size());
    for (std::size_t pt = 0; pt < graph_data.size(); pt++)
    {
        for (const JointPositions& q : graph_data[pt])
        {
            nodes[pt].push_back(std::make_shared<Node>(q, state_cost_fun(task_space_regions[pt], q)));
        }
    }

    // convert the path cost function to something that can work with NodePtr instead of JointPositions
    auto path_cost = [path_cost_fun](NodePtr n1, NodePtr n2) { return path_cost_fun(n1->data, n2->data); };

    // Find the shortest path in this structured array of nodes
    auto path_nodes = shortest_path_dag(nodes, path_cost);

    std::vector<JointPositions> path;
    for (NodePtr n : path_nodes)
    {
        path.push_back(n->data);
        std::cout << "Node: " << (*n) << " dist: " << n->dist << "\n";
    }

    if (path_nodes.size() < task_space_regions.size())
    {
        return Solution{ false, path };
    }
    else
    {
        return Solution{ true, path, path_nodes.back()->dist };
    }
}  // namespace ocpl

}  // namespace ocpl
