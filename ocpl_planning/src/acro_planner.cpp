#include <ocpl_planning/acro_planner.h>

#include <ocpl_graph/graph.h>
#include <ocpl_planning/cost_functions.h>
#include <ocpl_planning/io.h>

#include <algorithm>
#include <iostream>
#include <chrono>

namespace ocpl
{
std::vector<std::vector<ocpl_graph::NodePtr>> UnifiedPlanner::createGlobalRoadmap(
    const std::vector<TSR>& task_space_regions, const JointLimits& redundant_joint_limits,
    std::function<double(const TSR&, const JointPositions&)> state_cost_fun)
{
    auto path_samplers = createGlobalWaypointSamplers(task_space_regions, redundant_joint_limits);

    // sample valid joint positions along the path
    std::vector<std::vector<JointPositions>> graph_data;
    if (settings_.sampler_type == SamplerType::GRID)
    {
        graph_data = sampleGlobalGrid(path_samplers);
    }
    else
    {
        auto start = std::chrono::steady_clock::now();
        graph_data = sampleGlobalIncremental(path_samplers, settings_.min_valid_samples, settings_.max_iters);
        auto stop = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = stop - start;
        std::cout << "Sampling time: " << elapsed_seconds.count() << "\n";
    }

    // Convert the end-effector poses to Nodes, which is the desired output format for the getNeighbor function
    // can also be done in parallel if state cost function is heavy
    std::vector<std::vector<ocpl_graph::NodePtr>> nodes;
    nodes.resize(path_samplers.size());
    for (std::size_t pt = 0; pt < graph_data.size(); pt++)
    {
        for (const JointPositions& q : graph_data[pt])
        {
            nodes[pt].push_back(std::make_shared<ocpl_graph::Node>(q, state_cost_fun(task_space_regions[pt], q)));
        }
    }

    // Give the nodes correct waypoint indices.
    for (std::size_t index{ 0 }; index < nodes.size(); ++index)
    {
        for (auto& n : nodes[index])
            n->waypoint_index = index;
    }

    return nodes;
}

/************************************************************************
 * SOLVE
 * **********************************************************************/
Solution UnifiedPlanner::_solve(const std::vector<TSR>& task_space_regions, const JointLimits& redundant_joint_limits,
                                std::function<double(const JointPositions&, const JointPositions&)> path_cost_fun,
                                std::function<double(const TSR&, const JointPositions&)> state_cost_fun)
{
    if (settings_.is_redundant && redundant_joint_limits.empty())
        std::cout << "The robot is marked redundant, but the joint limits for the redundant joints are not "
                     "specified...\n";

    if (!settings_.is_redundant && redundant_joint_limits.size() > 0)
        std::cout << "The robot is not marked redundant, but joint limits for redundant joints where given.\n";

    initializeTaskSpaceSamplers(task_space_regions.at(0).bounds.asVector());

    // convert the path cost function to something that can work with NodePtr instead of JointPositions
    auto path_cost = [path_cost_fun](ocpl_graph::NodePtr n1, ocpl_graph::NodePtr n2) {
        return path_cost_fun(n1->data, n2->data);
    };

    std::vector<ocpl_graph::NodePtr> path_nodes;
    if (settings_.type == PlannerType::GLOBAL || settings_.type == PlannerType::GLOBAL_DFS)
    {
        auto nodes = createGlobalRoadmap(task_space_regions, redundant_joint_limits, state_cost_fun);

        // Wrap this nodes in a nearest getNeighbor function that the graph search needs
        auto sample_f = [&nodes](const ocpl_graph::NodePtr& node) {
            return nodes.at(node->waypoint_index + 1);
        };

        ocpl_graph::Graph graph(sample_f, nodes.size(), nodes.at(0));

        if (settings_.type == PlannerType::GLOBAL)
        {
            auto d_fun = [](const ocpl_graph::NodePtr& a, const ocpl_graph::NodePtr& b) { return b->dist < a->dist; };
            ocpl_graph::PriorityQueueContainer<ocpl_graph::NodePtr> container(graph.size(), d_fun);
            path_nodes = shortest_path_dag(graph, path_cost, container);
        }
        else /* if (settings_.type == PlannerType::GLOBAL_DFS) */
        {
            ocpl_graph::StackContainer<ocpl_graph::NodePtr> container;
            path_nodes = shortest_path_dag(graph, path_cost, container);
        }
    }
    else
    {
        // create local biased samplers for all path points
        std::vector<std::function<IKSolution(const JointPositions&)>> local_samplers;
        for (size_t wp{ 0 }; wp < task_space_regions.size(); ++wp)
        {
            local_samplers.push_back([wp, task_space_regions, this](const JointPositions& q_bias) {
                return sample(wp, q_bias, task_space_regions);
            });
        }

        auto sample_f = [local_samplers, state_cost_fun, task_space_regions, this](const ocpl_graph::NodePtr& node) {
            auto q_samples = sampleLocalIncremental(node->data, local_samplers[node->waypoint_index + 1],
                                                    settings_.min_valid_samples, settings_.max_iters);
            std::vector<ocpl_graph::NodePtr> nodes;
            for (const JointPositions& q : q_samples)
            {
                nodes.push_back(
                    std::make_shared<ocpl_graph::Node>(q, state_cost_fun(task_space_regions[node->waypoint_index], q)));
                nodes.back()->waypoint_index = node->waypoint_index + 1;
            }
            if (debug_)
            {
                std::cout << "locally sampling point " << node->waypoint_index << ". Found " << nodes.size()
                          << " samples\n";
            }
            return nodes;
        };

        // size_t old_mvs = settings_.min_valid_samples;
        // if (true)
        // {
        //     settings_.min_valid_samples = settings_.min_shots;
        // }
        std::vector<ocpl_graph::NodePtr> start_nodes =
            (createGlobalRoadmap({ task_space_regions.at(0) }, redundant_joint_limits, state_cost_fun)).at(0);
        // if (true)
        // {
        //     settings_.min_valid_samples = old_mvs;
        // }

        if (debug_)
        {
            std::cout << "local planner found " << start_nodes.size() << " start nodes.\n";
        }

        if (settings_.type == PlannerType::LOCAL_DFS)
        {
            ocpl_graph::Graph graph(sample_f, task_space_regions.size(), start_nodes);
            ocpl_graph::StackContainer<ocpl_graph::NodePtr> container;
            path_nodes = ocpl_graph::shortest_path_dag(graph, path_cost, container);
        }
        else /*if (settings_.type == PlannerType::LOCAL_BEST_FIRST_DFS) */
        {
            ocpl_graph::Graph graph(sample_f, task_space_regions.size(), start_nodes);
            auto d_fun = [](const ocpl_graph::NodePtr& a, const ocpl_graph::NodePtr& b) { return b->dist < a->dist; };
            ocpl_graph::PriorityStackContainer<ocpl_graph::NodePtr> container(graph.size(), d_fun);
            path_nodes = ocpl_graph::shortest_path_dag(graph, path_cost, container);
        }
    }

    std::vector<JointPositions> path;
    for (ocpl_graph::NodePtr n : path_nodes)
    {
        path.push_back(n->data);
        if (debug_)
            std::cout << "Node: " << (*n) << " dist: " << n->dist << "\n";
    }

    if (debug_)
    {
        for (size_t i{ 1 }; i < path.size(); ++i)
        {
            std::vector<double> diff(robot_.num_dof);
            for (size_t j{ 0 }; j < robot_.num_dof; ++j)
            {
                diff[j] = std::abs(path[i][j] - path[i - 1][j]);
            }
            std::cout << diff << "\n";
        }
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
    return _solve(task, red_joint_limits, norm2Diff, zeroStateCost);
}

Solution UnifiedPlanner::solve(const std::vector<TSR>& task,
                               std::function<double(const JointPositions&, const JointPositions&)> path_cost_fun,
                               std::function<double(const TSR&, const JointPositions&)> state_cost_fun)
{
    std::vector<Bounds> red_joint_limits(robot_.joint_limits.begin(), robot_.joint_limits.begin() + robot_.num_red_dof);
    return _solve(task, red_joint_limits, path_cost_fun, state_cost_fun);
}

}  // namespace ocpl
