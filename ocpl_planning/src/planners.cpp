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

/************************************************************************
 * GLOBAL SAMPLING
 * **********************************************************************/
std::vector<std::vector<JointPositions>> UnifiedPlanner::sampleGlobalIncremental(
    std::vector<std::function<IKSolution()>> path_samplers)
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

/************************************************************************
 * SOLVE
 * **********************************************************************/
Solution UnifiedPlanner::_solve(const std::vector<TSR>& task_space_regions, const JointLimits& redundant_joint_limits,
                                std::function<double(const JointPositions&, const JointPositions&)> path_cost_fun,
                                std::function<double(const TSR&, const JointPositions&)> state_cost_fun)
{
    // create the path samplers
    std::vector<std::function<IKSolution()>> path_samplers;

    if (settings_.is_redundant && redundant_joint_limits.empty())
        std::cout << "The robot is marked redundant, but the joint limits for the redundant joints are not "
                     "specified...\n";

    if (!settings_.is_redundant && redundant_joint_limits.size() > 0)
        std::cout << "The robot is not marked redundant, but joint limits for redundant joints where given.\n";

    if (settings_.is_redundant)
    {
        for (auto tsr : task_space_regions)
        {
            SamplerPtr t_sampler = createSampler(tsr.bounds.asVector(), settings_.sampler_type, settings_.tsr_resolution);
            SamplerPtr c_sampler =
                createSampler(redundant_joint_limits, settings_.sampler_type, settings_.redundant_joints_resolution);
            path_samplers.push_back(
                [tsr, redundant_joint_limits, t_sampler, c_sampler, this]() {
                    IKSolution result;
                    for (auto tsr_values : t_sampler->getSamples(settings_.t_space_batch_size))
                    {
                        for (auto q_red : c_sampler->getSamples(settings_.c_space_batch_size))
                        {
                            for (auto q : robot_.ik(tsr.valuesToPose(tsr_values), q_red))
                            {
                                if (robot_.isValid(q))
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
            SamplerPtr sampler = createSampler(tsr.bounds.asVector(), settings_.sampler_type, settings_.tsr_resolution);
            path_samplers.push_back([tsr, sampler, this]() {
                IKSolution result;
                for (auto tsr_values : sampler->getSamples(settings_.t_space_batch_size))
                {
                    for (auto q : robot_.ik(tsr.valuesToPose(tsr_values), {}))
                    {
                        if (robot_.isValid(q))
                            result.push_back(q);
                    }
                }
                return result;
            });
        }
    }

    // sample valid joint positions along the path
    std::vector<std::vector<JointPositions>> graph_data;
    if (settings_.sampler_type == SamplerType::GRID)
    {
        graph_data = sampleGlobalGrid(path_samplers);
    }
    else
    {
        auto start = std::chrono::steady_clock::now();
        graph_data = sampleGlobalIncremental(path_samplers);
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

    // Give the nodes correct waypoint indices.
    for (std::size_t index{ 0 }; index < nodes.size(); ++index)
    {
        for (auto& n : nodes[index])
            n->waypoint_index = index;
    }

    // convert the path cost function to something that can work with NodePtr instead of JointPositions
    auto path_cost = [path_cost_fun](NodePtr n1, NodePtr n2) { return path_cost_fun(n1->data, n2->data); };

    // DAGraph graph(nodes);

    // test out tree
    auto sample_f = [&nodes](const NodePtr& node, size_t /* num_samples */) {
        return nodes.at(node->waypoint_index + 1);
    };
    Tree graph(sample_f, nodes.size(), nodes.at(0));

    // Find the shortest path in this structured array of nodes
    auto path_nodes = shortest_path_dag(graph, path_cost);

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
}

Solution UnifiedPlanner::solve(const std::vector<TSR>& task)
{
    std::vector<Bounds> red_joint_limits(robot_.joint_limits.begin(), robot_.joint_limits.begin() + robot_.num_red_dof);
    return _solve(task, red_joint_limits, L2NormDiff2, zeroStateCost);
}

}  // namespace ocpl
