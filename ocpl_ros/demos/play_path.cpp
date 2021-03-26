#include <ros/ros.h>

#include <algorithm>
#include <limits>
#include <chrono>

#include <simple_moveit_wrapper/industrial_robot.h>

#include <ocpl_ros/rviz.h>

#include <ocpl_planning/io.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

using namespace ocpl;

trajectory_msgs::JointTrajectory convertPath(std::vector<JointPositions> path,
                                             const std::vector<std::string>& joint_names)
{
    trajectory_msgs::JointTrajectory traj;
    traj.joint_names = joint_names;

    double time = 0.0;
    for (auto q : path)
    {
        trajectory_msgs::JointTrajectoryPoint pt;
        pt.positions = q;
        pt.velocities.resize(q.size(), 0.0);
        pt.accelerations.resize(q.size(), 0.0);
        pt.effort.resize(q.size(), 0.0);
        pt.time_from_start = ros::Duration(time);
        time += 0.1;
        traj.points.push_back(pt);
    }
    return traj;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ocpl_moveit_demo_rn");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    auto display_publisher =
        node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

    // std::shared_ptr<MoveItRobot> robot = std::make_shared<PlanarRobotNR>();
    std::shared_ptr<simple_moveit_wrapper::Robot> robot =
        std::make_shared<simple_moveit_wrapper::IndustrialRobot>("manipulator", "tool_tip");

    simple_moveit_wrapper::Robot table("rotation_table", "table");
    std::vector<std::string> joint_names = table.getJointNames();

    auto robot_joint_names = robot->getJointNames();
    joint_names.insert(joint_names.end(), robot_joint_names.begin(), robot_joint_names.end());

    Rviz rviz;
    ros::Duration(0.2).sleep();
    // rviz.clear();
    // ros::Duration(0.2).sleep();

    std::string filename{ "last_path.npy" };

    if (argc > 1)
    {
        filename = std::string(argv[1]);
    }
    std::cout << "Reading path from file: " << filename << "\n";

    auto path = loadPath(filename);

    robot->animatePath(rviz.visual_tools_, path);

    // auto ros_traj = convertPath(path, joint_names);

    // moveit_msgs::RobotTrajectory robot_traj;
    // robot_traj.joint_trajectory = ros_traj;

    // // rviz.visual_tools_->publishTrajectoryPath(robot_traj, robot->getMoveItRobotState());

    // moveit_msgs::DisplayTrajectory display_trajectory;
    // display_trajectory.trajectory.push_back({});
    // display_trajectory.trajectory.back().joint_trajectory = ros_traj;

    // // Use the default planning scene published by the move group node.
    // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // auto psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
    // bool has_planning_scene = psm->requestPlanningSceneState("/get_planning_scene");
    // ROS_INFO_STREAM("Request planning scene " << (has_planning_scene ? "succeeded." : "failed."));
    // psm->startSceneMonitor("/move_group/monitored_planning_scene");
    // psm->getPlanningScene()->printKnownObjects();

    // // auto attached_objects = psm->getPlanningScene()->getAttachedCollisionObjectMsgs();
    // // // for (auto it = attached_objects.begin(); it != attached_objects.end(); ++it)
    // // // {
    // // //     display_trajectory.trajectory_start.attached_collision_objects.push_back(it->second);
    // // // }

    // std::vector<moveit_msgs::AttachedCollisionObject> aco;
    // psm->getPlanningScene()->getAttachedCollisionObjectMsgs(aco);
    // for (auto ob : aco)
    // {
    //     ROS_INFO_STREAM(ob);
    //     display_trajectory.trajectory_start.attached_collision_objects.push_back(ob);
    // }

    // display_publisher.publish(display_trajectory);

    ros::shutdown();

    return 0;
}
