#include <ocpl_planning/planners.h>

#include <ocpl_graph/tree.h>

#include <iostream>

namespace ocpl
{
// std::vector<JointPositions> sampleTSR(const TSR& tsr, std::function<bool(const JointPositions&)> is_valid,
//                                       std::function<IKSolution(const Transform&)> generic_inverse_kinematics)
// {
//     std::vector<JointPositions> valid_ik_solutions;
//     auto samples = tsr.getSamples();
//     for (Transform& tf : samples)
//     {
//         for (JointPositions& q : generic_inverse_kinematics(tf))
//         {
//             if (is_valid(q))
//                 valid_ik_solutions.push_back(q);
//         }
//     }
//     return valid_ik_solutions;
// };

std::vector<JointPositions> sampleTSR(const TSR& tsr, std::function<bool(const JointPositions&)> is_valid,
                                      std::function<IKSolution(const TSR&)> generic_inverse_kinematics)
{
    std::vector<JointPositions> valid_ik_solutions;
    for (JointPositions& q : generic_inverse_kinematics(tsr))
    {
        if (is_valid(q))
            valid_ik_solutions.push_back(q);
    }
    return valid_ik_solutions;
};

std::vector<JointPositions> findPath(const std::vector<TSR>& tsrs,
                                     std::function<double(const NodePtr, const NodePtr)> cost_function,
                                     std::function<double(const TSR&, const JointPositions&)> state_cost,
                                     std::function<bool(const JointPositions&)> is_valid,
                                     std::function<IKSolution(const TSR&)> generic_inverse_kinematics)
{
    // calculate joint positions for all path points in Cartesian space
    std::vector<std::vector<JointPositions>> graph_data;
    for (auto tsr : tsrs)
    {
        std::cout << "ocpl_planner: processing path point " << tsr.tf_nominal.translation().transpose() << "\n";
        auto solutions = sampleTSR(tsr, is_valid, generic_inverse_kinematics);
        std::cout << "ocpl_planner: found " << solutions.size() << std::endl;
        graph_data.push_back(solutions);
    }

    // Convert the end-effector poses to Nodes, so we can search for the shortest path
    std::vector<std::vector<NodePtr>> nodes;
    nodes.resize(tsrs.size());
    for (std::size_t pt = 0; pt < graph_data.size(); pt++)
    {
        for (const JointPositions& q : graph_data[pt])
        {
            nodes[pt].push_back(std::make_shared<Node>(q, state_cost(tsrs[pt], q)));
        }
    }

    // Find the shortest path in this structured array of nodes
    auto path_nodes = shortest_path_dag(nodes, cost_function);
    std::vector<JointPositions> path;
    for (NodePtr n : path_nodes)
    {
        path.push_back(n->data);
        std::cout << "Node: " << (*n) << " dist: " << n->dist << "\n";
    }
    return path;
}

std::vector<JointPositions> findPath(const std::vector<std::function<IKSolution(int)>>& path_samplers,
                                     const std::vector<Transform>& nominal_tfs,
                                     std::function<double(const NodePtr, const NodePtr)> cost_function,
                                     std::function<double(const Transform&, const JointPositions&)> state_cost,
                                     std::function<bool(const JointPositions&)> is_valid)
{
    // calculate joint positions for all path points in Cartesian space
    std::vector<std::vector<JointPositions>> graph_data;
    int path_count{ 0 };
    for (auto path_sampler : path_samplers)
    {
        std::cout << "ocpl_planner: processing path point " << path_count << "\n";
        std::vector<JointPositions> valid_q;
        for (JointPositions& q : path_sampler(10))
        {
            if (is_valid(q))
                valid_q.push_back(q);
        }
        std::cout << "ocpl_planner: found " << valid_q.size() << std::endl;
        graph_data.push_back(valid_q);
        path_count++;
    }

    // Convert the end-effector poses to Nodes, so we can search for the shortest path
    std::vector<std::vector<NodePtr>> nodes;
    nodes.resize(path_samplers.size());
    for (std::size_t pt = 0; pt < graph_data.size(); pt++)
    {
        for (const JointPositions& q : graph_data[pt])
        {
            nodes[pt].push_back(std::make_shared<Node>(q, state_cost(nominal_tfs[pt], q)));
        }
    }

    // Find the shortest path in this structured array of nodes
    auto path_nodes = shortest_path_dag(nodes, cost_function);
    std::vector<JointPositions> path;
    for (NodePtr n : path_nodes)
    {
        path.push_back(n->data);
        std::cout << "Node: " << (*n) << " dist: " << n->dist << "\n";
    }
    return path;
}

}  // namespace ocpl
