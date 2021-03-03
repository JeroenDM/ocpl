#include <ocpl_planning/acro_planner.h>

#include <ocpl_graph/tree.h>
#include <ocpl_planning/cost_functions.h>

#include <algorithm>
#include <iostream>
#include <chrono>

namespace ocpl
{
std::vector<std::vector<NodePtr>> UnifiedPlanner::createGlobalRoadmap(
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
        graph_data = sampleGlobalIncremental(path_samplers);
        auto stop = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = stop - start;
        std::cout << "Sampling time: " << elapsed_seconds.count() << "\n";
    }

    // Convert the end-effector poses to Nodes, which is the desired output format for the getNeighbor function
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
    auto path_cost = [path_cost_fun](NodePtr n1, NodePtr n2) { return path_cost_fun(n1->data, n2->data); };

    std::vector<NodePtr> path_nodes;
    if (settings_.type == PlannerType::GLOBAL)
    {
        auto nodes = createGlobalRoadmap(task_space_regions, redundant_joint_limits, state_cost_fun);

        // Wrap this nodes in a nearest getNeighbor function that the graph search needs
        auto sample_f = [&nodes](const NodePtr& node, size_t /* num_samples */) {
            return nodes.at(node->waypoint_index + 1);
        };
        Tree graph(sample_f, nodes.size(), nodes.at(0));

        // Find the shortest path in this structured array of nodes
        path_nodes = shortest_path_dag(graph, path_cost);
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

        auto sample_f = [local_samplers, state_cost_fun, task_space_regions, this](const NodePtr& node,
                                                                                   size_t /* num_samples */) {
            auto q_samples = sampleLocalIncremental(node->data, local_samplers[node->waypoint_index + 1]);
            std::vector<NodePtr> nodes;
            for (const JointPositions& q : q_samples)
            {
                nodes.push_back(std::make_shared<Node>(q, state_cost_fun(task_space_regions[node->waypoint_index], q)));
                nodes.back()->waypoint_index = node->waypoint_index + 1;
            }
            std::cout << "locally sampling point " << node->waypoint_index << ". Found " << nodes.size() << " samples\n";
            return nodes;
        };

        std::vector<NodePtr> start_nodes =
            (createGlobalRoadmap({ task_space_regions.at(0) }, redundant_joint_limits, state_cost_fun)).at(0);
        std::cout << "local planner found " << start_nodes.size() << " start nodes.\n";
        Tree graph(sample_f, task_space_regions.size(), start_nodes);

        // Find the shortest path in this structured array of nodes
        path_nodes = shortest_path_dag(graph, path_cost);
    }

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
