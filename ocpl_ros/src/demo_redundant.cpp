#include <ros/ros.h>

#include <cmath>
#include <algorithm>

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

    std::vector<double> q_start = { 1.0, -1.0, 0.3, -0.4, 0.5, -0.6 };
    auto tf_start = robot.fk(q_start);
    auto solution = robot.ik(tf_start, {1.0, -1.0, 0.3});

    std::cout << tf_start.translation().transpose() << std::endl;

    rviz.plotPose(tf_start);
    robot.plot(rviz.visual_tools_, q_start);

    ROS_INFO_STREAM("Found " << solution.size() << " ik solutions.");
    for (auto q : solution)
    {
        robot.plot(rviz.visual_tools_, q, rviz_visual_tools::GREEN);
        ros::Duration(0.5).sleep();
    }

    return 0;
}
