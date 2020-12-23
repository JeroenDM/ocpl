#include <ros/ros.h>

#include <cmath>
#include <algorithm>
#include <limits>

#include <ocpl_ros/moveit_robot_examples.h>
#include <ocpl_ros/rviz.h>
#include <ocpl_sampling/grid_sampler.h>
#include <ocpl_tsr/task_space_regions.h>
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

double L1NormDiff(NodePtr n1, NodePtr n2)
{
    double cost{};
    int s = n1->data.size();
    for (int i = 0; i < n1->data.size(); ++i)
    {
        cost += std::abs(n1->data[i] - n2->data[i]);
    }
    // if (cost > 1.0)
    //     cost = std::numeric_limits<double>::max();
    return cost;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ocpl_moveit_demo_r6");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    PlanarRobot6R robot;
    Rviz rviz;
    ros::Duration(0.2).sleep();
    rviz.clear();
    ros::Duration(0.2).sleep();

    std::vector<double> q_start = { 1.0, -2.0, 1.0, -1.0, 1.0, -1.0 };
    auto tf_start = robot.fk(q_start);
    auto solution = robot.ik(tf_start, { 1.0, -1.0, 0.3 });

    std::cout << tf_start.translation().transpose() << std::endl;

    rviz.plotPose(tf_start);
    robot.plot(rviz.visual_tools_, q_start);

    ROS_INFO_STREAM("Found " << solution.size() << " ik solutions.");
    for (auto q : solution)
    {
        robot.plot(rviz.visual_tools_, q, rviz_visual_tools::GREEN);
        ros::Duration(0.5).sleep();
    }

    auto sampler = std::make_shared<GridSampler>();
    sampler->addDimension(-1.5, 1.5, 6);
    sampler->addDimension(-1.5, 1.5, 6);
    sampler->addDimension(-1.5, 1.5, 6);

    // for (auto q_red : sampler->getSamples())
    // {
    //     auto solution = robot.ik(tf_start, q_red);
    //     ROS_INFO_STREAM("Found " << solution.size() << " ik solutions.");
    //     for (auto q : solution)
    //     {
    //         robot.plot(rviz.visual_tools_, q, rviz_visual_tools::GREEN);
    //         ros::Duration(0.1).sleep();
    //     }
    // }

    //////////////////////////////////
    // Create task
    //////////////////////////////////
    const double small_passage_width {0.5};
    Transform tf1 = Transform::Identity();
    tf1.translation() << 4.0, small_passage_width / 2, tf_start.translation().z();
    TSRBounds bounds{ { 0.0, 0.0 }, { 0.0, 0.0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, M_PI } };
    std::vector<TSR> regions;

    Transform tf = tf1;
    for (int i{ 0 }; i < 5; ++i)
    {
        tf.translation() += 0.2 * Eigen::Vector3d::UnitX();
        regions.push_back({ tf, bounds, std::make_shared<GridSampler>(), { 1, 1, 1, 1, 1, 30 } });
    }

    for (auto tsr : regions)
    {
        rviz.plotPose(tsr.getNominalPose());
        ros::Duration(0.05).sleep();
    }

    //////////////////////////////////
    // Describe problem
    //////////////////////////////////
    auto f_is_valid = [&robot](const JointPositions& q) { return !robot.isInCollision(q); };
    auto f_generic_inverse_kinematics = [&robot, &sampler](const Transform& tf) {
        IKSolution result;
        for (auto q_red : sampler->getSamples())
        {
            auto solution = robot.ik(tf, q_red);
            for (auto q : solution)
            {
                result.push_back(q);
            }
        }
        return result;
    };

    // appart from edge costs, the graph also used node costs
    // double state_weight{ 1.0 };
    // auto f_state_cost = [&robot, state_weight](const TSR& tsr, const JointPositions& q) {
    //     double z0 = tsr.getNominalPose().rotation().eulerAngles(0, 1, 2).z();
    //     double z1 = robot.fk(q).rotation().eulerAngles(0, 1, 2).z();
    //     return state_weight * (z0 - z1) * (z0 - z1);
    //     // return std::abs(z1 - 0.3);
    // };

    auto f_state_cost = [](const TSR& /* tsr */, const JointPositions& /* q */) { return 0.0; };

    //////////////////////////////////
    // Solve problem
    //////////////////////////////////
    auto path = findPath(regions, L1NormDiff, f_state_cost, f_is_valid, f_generic_inverse_kinematics);

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
