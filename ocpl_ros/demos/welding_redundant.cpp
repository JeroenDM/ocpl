#include <ros/ros.h>

#include <algorithm>
#include <limits>
#include <chrono>

#include <ocpl_ros/moveit_robot_examples.h>
#include <ocpl_ros/rviz.h>
#include <ocpl_sampling/grid_sampler.h>
#include <ocpl_sampling/halton_sampler.h>
#include <ocpl_sampling/random_sampler.h>
#include <ocpl_tsr/task_space_regions.h>

#include <ocpl_planning/planners.h>
#include <ocpl_planning/factories.h>
#include <ocpl_planning/cost_functions.h>

using namespace ocpl;
using Eigen::AngleAxisd;
using Eigen::Isometry3d;
using Eigen::Vector3d;

std::vector<TSR> createLineTask(TSRBounds bounds, Eigen::Vector3d start, Eigen::Vector3d stop,
                                Eigen::Isometry3d orientation, std::size_t num_points)
{
    std::vector<TSR> task;
    Transform tf(orientation);
    tf.translation() = start;

    Eigen::Vector3d direction = (stop - start).normalized();

    double step = (stop - start).norm() / num_points;
    for (int i{ 0 }; i < num_points; ++i)
    {
        tf.translation() += step * direction;
        task.push_back({ tf, bounds });
    }
    return task;
}

void showPath(const std::vector<JointPositions>& path, Rviz& rviz, MoveItRobot& robot)
{
    for (JointPositions q : path)
    {
        if (robot.isInCollision(q))
            robot.plot(rviz.visual_tools_, q, rviz_visual_tools::RED);
        else
            robot.plot(rviz.visual_tools_, q, rviz_visual_tools::GREEN);
        ros::Duration(0.1).sleep();
    }
}

inline double deg2rad(double deg)
{
    return deg * M_PI / 180.0;
}

/** \brief Demo for a planar, 6 link robot.
 *
 * You can specify and load collision objects with the python script "load_collision_objects.py".
 * The main function shows how to setup and solve a task of following a simple straight line.
 *
 * */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "ocpl_moveit_demo_rn");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    IndustrialRobot robot;
    Rviz rviz;
    ros::Duration(0.2).sleep();
    rviz.clear();
    ros::Duration(0.2).sleep();

    // create an interesting start configurations
    std::vector<double> q_start(robot.getNumDof(), 1.0);

    auto tf_start = robot.fk(q_start);
    // std::cout << tf_start.translation().transpose() << std::endl;

    rviz.plotPose(tf_start);
    robot.plot(rviz.visual_tools_, q_start, rviz_visual_tools::MAGENTA);
    ros::Duration(0.2).sleep();


    // for (auto q_red : sampler->getSamples())
    // {
    auto solution = robot.ik(tf_start, {0.5});
    ROS_INFO_STREAM("Found " << solution.size() << " ik solutions.");
    for (auto q : solution)
    {
        robot.plot(rviz.visual_tools_, q, rviz_visual_tools::GREEN);
        ros::Duration(0.2).sleep();
    }
    // }

    //////////////////////////////////
    // Create task
    //////////////////////////////////
    TSRBounds bounds{ { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { -M_PI, M_PI } };
    Vector3d work(2, 0.3, 0.9027);
    Vector3d start(0.0, 1.05, 0.05);
    Vector3d stop(0.9, 1.05, 0.05);
    Isometry3d ori(AngleAxisd(deg2rad(135), Vector3d::UnitX()) * AngleAxisd(0.0, Vector3d::UnitY()) *
                   AngleAxisd(0.0, Vector3d::UnitZ()));
    auto regions = createLineTask(bounds, start + work, stop + work, ori, 10);

    // add another line
    Vector3d start_2(0.95, 1.1, 0.05);
    Vector3d stop_2(0.95, 2.9, 0.05);
    Isometry3d ori_2(AngleAxisd(0.0, Vector3d::UnitX()) * AngleAxisd(deg2rad(135), Vector3d::UnitY()) *
                   AngleAxisd(deg2rad(90), Vector3d::UnitZ()));
    
    auto second_line = createLineTask(bounds, start_2 + work, stop_2 + work, ori_2, 20);
    regions.insert(regions.end(), second_line.begin(), second_line.end());


    JointLimits joint_limits{{0.0, 3.0}};

    for (TSR& tsr : regions)
    {
        rviz.plotPose(tsr.tf_nominal);
        ros::Duration(0.05).sleep();
    }

    //////////////////////////////////
    // Simple interface solver
    //////////////////////////////////
    auto redundant_joint_limits = joint_limits;

    // function that tells you whether a state is valid (collision free)
    auto is_valid_fun = [&robot](const JointPositions& q) { return !robot.isInCollision(q); };

    // function that returns analytical inverse kinematics solution for end-effector pose
    auto ik_fun = [&robot](const Transform& tf, const JointPositions& q_fixed) { return robot.ik(tf, q_fixed); };

    // specify an objective to minimize the cost along the path
    // here we use a predefined function that uses the L1 norm of the different
    // between two joint positions
    auto path_cost_fun = L1NormDiff2;

    // optionally we can set a state cost for every point along the path
    // this one tries to keep the end-effector pose close the the nominal pose
    // defined in the task space region
    // auto state_cost_fun = [&robot](const TSR& tsr, const JointPositions& q) {
    //     return poseDistance(tsr.tf_nominal, robot.fk(q)).norm();
    // };
    auto state_cost_fun = zeroStateCost;

    // settings to select a planner
    PlannerSettings ps;
    ps.is_redundant = true;
    // ps.sampler_type = SamplerType::HALTON;
    // ps.t_space_batch_size = 10;
    // ps.c_space_batch_size = 100;
    // ps.min_valid_samples = 50;
    // ps.max_iters = 200;

    ps.sampler_type = SamplerType::GRID;
    ps.tsr_resolution = { 1, 1, 1, 1, 1, 30 };
    ps.redundant_joints_resolution = std::vector<int> {30};

    // solve it!
    Solution res = solve(regions, joint_limits, ik_fun, is_valid_fun, path_cost_fun, state_cost_fun, ps);
    if (res.success)
    {
        std::cout << "A solution is found with a cost of " << res.cost << "\n";
    }
    else
    {
        std::cout << "No complete solution was found.\n";
    }

    showPath(res.path, rviz, robot);

    return 0;
}
