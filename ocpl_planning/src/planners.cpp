#include <ocpl_planning/planners.h>

#include <ocpl_graph/tree.h>
#include <ocpl_planning/cost_functions.h>
#include <ocpl_planning/factories.h>

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

std::vector<JointPositions> solve(const std::vector<TSR>& task_space_regions,
                                  std::function<IKSolution(const Transform&, const JointPositions&)> ik_fun,
                                  std::function<bool(const JointPositions&)> is_valid_fun,
                                  std::function<double(const JointPositions&, const JointPositions&)> path_cost_fun,
                                  std::function<double(const TSR&, const JointPositions&)> state_cost_fun,
                                  PlannerSettings ps)
{
    SamplerType sampler_type;
    if (ps.sample_method == SampleMethod::INCR_DET)
    {
        sampler_type = SamplerType::HALTON;
    }
    else
    {
        sampler_type = SamplerType::RANDOM;
    }

    // create the path samplers
    std::vector<std::function<IKSolution(int, int)>> path_samplers;

    if (ps.is_redundant)
    {
        std::cout << "Not implemented yet" << std::endl;
        exit(1);
        // for (auto tsr : task_space_regions)
        // {
        //     path_samplers.push_back(
        //         [tsr, ik_fun, is_valid_fun, sampler_type](int num_t_samples, int num_c_samples) {
        //             static SamplerPtr t_sampler = createIncrementalSampler(tsr, sampler_type);
        //             static SamplerPtr c_sampler = createIncrementalSampler(tsr, sampler_type);

        //             IKSolution result;
        //             for (auto sample : t_sampler->getSamples(num_t_samples))
        //             {
        //                 for (auto q_red : c_sampler->getSamples(num_c_samples))
        //                 {
        //                     for (auto q : ik_fun(tsr.valuesToPose(sample), q_red))
        //                     {
        //                         if (is_valid_fun(q))
        //                             result.push_back(q);
        //                     }
        //                 }
        //             }

        //             return result;
        //         });
        // }
    }  // namespace ocpl
    else
    {
        for (auto tsr : task_space_regions)
        {
            path_samplers.push_back(
                [tsr, ik_fun, is_valid_fun, sampler_type](int num_t_samples, int /* num_c_samples */) {
                    static SamplerPtr sampler = createIncrementalSampler(tsr, sampler_type);

                    IKSolution result;
                    for (auto sample : sampler->getSamples(num_t_samples))
                    {
                        for (auto q : ik_fun(tsr.valuesToPose(sample), {}))
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
    // TODO run this in parallel
    std::vector<std::vector<JointPositions>> graph_data;
    int path_count{ 0 };
    for (auto path_sampler : path_samplers)
    {
        std::cout << "ocpl_planner: processing path point " << path_count << "\n";

        int iters{ 0 };
        std::vector<JointPositions> valid_samples;
        while (iters < ps.max_iters && valid_samples.size() < ps.min_valid_samples)
        {
            auto s = path_sampler(ps.t_space_batch_size, ps.c_space_batch_size);

            // add the new joint positions to valid_samples (stl can be ugly...)
            valid_samples.reserve(valid_samples.size() + std::distance(s.begin(), s.end()));
            valid_samples.insert(valid_samples.end(), s.begin(), s.end());

            iters++;
        }
        graph_data.push_back(valid_samples);

        std::cout << "ocpl_planner: found " << valid_samples.size() << std::endl;

        path_count++;
    }

    // Convert the end-effector poses to Nodes, so we can search for the shortest path
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
    return path;
}  // namespace ocpl

}  // namespace ocpl
