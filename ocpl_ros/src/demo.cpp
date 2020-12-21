#include <ros/ros.h>

#include <cmath>
#include <algorithm>

#include <ocpl_ros/moveit_robot_examples.h>
#include <ocpl_ros/rviz.h>
#include <ocpl_sampling/grid_sampler.h>
#include <ocpl_tsr/task_space_regions.h>
// #include <ocpl_graph/tree.h>
#include <ocpl_planning/planners.h>

using namespace ocpl;

double L2NormDiff(NodePtr n1, NodePtr n2)
{
    double cost{};
    int s = n1->data.size();
    for (int i = 0; i < n1->data.size(); ++i)
    {
        cost += std::sqrt((n1->data[i] - n2->data[i]) * (n1->data[i] - n2->data[i]));
    }
    return cost;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ocpl_moveit_demo");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    PlanarRobot3R robot;
    Rviz rviz;
    ros::Duration(0.2).sleep();
    rviz.clear();
    ros::Duration(0.2).sleep();

    // std::vector<double> q_start = { 1.0, -0.7, 0 };
    std::vector<double> q_start = { 0.0, 0.0, 0.0 };
    auto tf_start = robot.fk(q_start);
    auto solution = robot.ik(tf_start);

    std::cout << tf_start.translation().transpose() << std::endl;

    rviz.plotPose(tf_start);
    robot.plot(rviz.visual_tools_, q_start);

    //////////////////////////////////
    // Create task
    //////////////////////////////////
    TSRBounds bounds{ { 0.0, 0.0 }, { 0.0, 0.0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { -1, 1 } };
    std::vector<TSR> regions;
    for (int i{ 0 }; i < 10; ++i)
    {
        Transform tf = tf_start;
        tf.translation().x() = 2.6;
        tf.translation().y() = static_cast<double>(i) / 10 - 0.5;
        regions.push_back({ tf, bounds, std::make_shared<GridSampler>(), { 1, 1, 1, 1, 1, 10 } });
    }

    //////////////////////////////////
    // Convert into joint space grid
    //////////////////////////////////
    // auto f_is_valid = [&robot](const JointPositions& q) { return !robot.isInCollision(q); };
    // auto f_generic_inverse_kinematics = [&robot](const Transform& tf) { return robot.ik(tf); };

    // std::vector<std::vector<JointPositions>> graph_data;
    // for (auto tsr : regions)
    // {
    //     graph_data.push_back(sampleTSR(tsr, f_is_valid, f_generic_inverse_kinematics));
    // }

    // double state_weight{ 1.0 };
    // auto f_state_cost = [&robot, state_weight](const TSR& tsr, const JointPositions& q) {
    //     double z0 = tsr.getNominalPose().rotation().eulerAngles(0, 1, 2).z();
    //     double z1 = robot.fk(q).rotation().eulerAngles(0, 1, 2).z();
    //     return state_weight * (z0 - z1) * (z0 - z1);
    //     // return std::abs(z1 - 0.3);
    // };

    // //////////////////////////////////
    // // Convert to nodes
    // //////////////////////////////////
    // // convert joint positions to nodes organized along the waypoints
    // std::vector<std::vector<NodePtr>> nodes;
    // nodes.resize(regions.size());
    // for (std::size_t pt = 0; pt < graph_data.size(); pt++)
    // {
    //     for (const JointPositions& q : graph_data[pt])
    //     {
    //         nodes[pt].push_back(std::make_shared<Node>(q, f_state_cost(regions[0], q)));
    //     }
    // }

    // //////////////////////////////////
    // // Run graph search
    // //////////////////////////////////
    // auto path = shortest_path(nodes, L2NormDiff);

    auto f_is_valid = [&robot](const JointPositions& q) { return !robot.isInCollision(q); };
    auto f_generic_inverse_kinematics = [&robot](const Transform& tf) { return robot.ik(tf); };
    double state_weight{ 1.0 };
    auto f_state_cost = [&robot, state_weight](const TSR& tsr, const JointPositions& q) {
        double z0 = tsr.getNominalPose().rotation().eulerAngles(0, 1, 2).z();
        double z1 = robot.fk(q).rotation().eulerAngles(0, 1, 2).z();
        return state_weight * (z0 - z1) * (z0 - z1);
        // return std::abs(z1 - 0.3);
    };

    auto path = findPath(regions, L2NormDiff, f_state_cost, f_is_valid, f_generic_inverse_kinematics);

    for (auto q : path)
    {
        if (robot.isInCollision(q))
            robot.plot(rviz.visual_tools_, q, rviz_visual_tools::RED);
        else
            robot.plot(rviz.visual_tools_, q, rviz_visual_tools::GREEN);
        ros::Duration(0.5).sleep();
    }

    return 0;
}

// visualize tsr sampling process
// for (auto tsr : regions)
// {
//     rviz.plotPose(tsr.getNominalPose());

//     std::vector<JointPositions> waypoint_data;
//     auto tfs = tsr.getSamples();
//     for (auto pose : tfs)
//     {
//         // rviz.plotPose(pose);
//         auto sol = robot.ik(pose);
//         for (auto q : sol)
//         {
//             if (is_valid(q))
//             {
//                 // robot.plot(rviz.visual_tools_, q);
//                 // ros::Duration(0.1).sleep();
//                 waypoint_data.push_back(q);
//             }
//         }
//     }
//     // graph_data.push_back(std::move(waypoint_data));
//     graph_data.push_back(waypoint_data);
// }

// // what is the nominal rotation around z?
// double z_nom = regions[0].getNominalPose().rotation().eulerAngles(0, 1, 2).z();
// std::cout << "Nominal z rotation: " << z_nom << "\n";

// // Construct a tree from the nodes
// Tree tree;
// // add the start nodes
// for (auto waypoint_nodes : nodes)
// {
//     for (auto node : waypoint_nodes)
//     {
//         tree[node] = {};
//     }
// }

// // connect all nodes of a waypoint to all the nodes of the next waypoint
// for (std::size_t pt = 0; pt < graph_data.size() - 1; pt++)
// {
//     for (const NodePtr& first : nodes[pt])
//     {
//         if (tree.find(first) != tree.end())
//         {
//             for (const NodePtr& second : nodes[pt + 1])
//             {
//                 tree[first].push_back(Edge(second, L2NormDiff(first, second)));
//             }
//         }
//         else
//         {
//             ROS_ERROR_STREAM("Node not found in tree: " << first);
//         }
//     }
// }

// auto path = shortest_path(tree, nodes.front(), nodes.back());

// template <typename T_IN, typename T_OUT>
// std::vector<std::vector<T_OUT>> vapply(const std::vector<std::vector<T_IN>>& v, std::function<T_OUT(T_IN)> fun)
// {
//     std::vector<std::vector<T_OUT>> result;
//     result.resize(v.size());
//     for (std::size_t row{ 0 }; row < v.size(); ++row)
//     {
//         result[row].resize(v[row].size());
//         for (std::size_t col{ 0 }; col < v[row].size(); ++col)
//         {
//             result[row][col] = fun(v[row][col]);
//         }
//     }
//     return result;
// }
