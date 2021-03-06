#include <ros/ros.h>

#include <algorithm>

#include <ocpl_ros/moveit_robot_examples.h>
#include <ocpl_ros/rviz.h>
#include <ocpl_sampling/grid_sampler.h>
#include <ocpl_sampling/halton_sampler.h>
#include <ocpl_sampling/random_sampler.h>
#include <ocpl_tsr/task_space_regions.h>

#include <ocpl_planning/planners.h>
#include <ocpl_planning/factories.h>
#include <ocpl_planning/cost_functions.h>
#include <ocpl_planning/types.h>

using namespace ocpl;

void showPath(const std::vector<JointPositions>& path, Rviz& rviz, PlanarRobot3R& robot)
{
    for (JointPositions q : path)
    {
        if (robot.isInCollision(q))
            robot.plot(rviz.visual_tools_, q, rviz_visual_tools::RED);
        else
            robot.plot(rviz.visual_tools_, q, rviz_visual_tools::GREEN);
        ros::Duration(0.5).sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ocpl_moveit_demo");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    PlanarRobot3R robot;
    Rviz rviz;
    rviz.clear();

    std::vector<double> q_start = { 1.0, -0.7, 0 };
    // std::vector<double> q_start = { 0.0, 0.0, 0.0 };
    auto tf_start = robot.fk(q_start);
    auto q_start_ik = robot.ik(tf_start);

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
        regions.push_back({ tf, bounds });
        rviz.plotPose(tf);
        ros::Duration(0.05).sleep();
    }

    // joint limits for the redundant joints
    JointLimits joint_limits{};

    //////////////////////////////////
    // Simple interface solver
    //////////////////////////////////

    // function that tells you whether a state is valid (collision free)
    auto f_is_valid = [&robot](const JointPositions& q) { return !robot.isInCollision(q); };

    // function that returns analytical inverse kinematics solution for end-effector pose
    auto f_ik = [&robot](const Transform& tf, const JointPositions& /*q_fixed*/) { return robot.ik(tf); };

    // specify an objective to minimize the cost along the path
    // here we use a predefined function that uses the L1 norm of the different
    // between two joint positions
    auto f_path_cost = L1NormDiff2;

    // optionally we can set a state cost for every point along the path
    // this one tries to keep the end-effector pose close the the nominal pose
    // defined in the task space region
    auto f_state_cost = [&robot](const TSR& tsr, const JointPositions& q) {
        return poseDistance(tsr.tf_nominal, robot.fk(q)).norm();
    };

    // settings to select a planner
    PlannerSettings ps;
    ps.sampler_type = SamplerType::HALTON;
    ps.t_space_batch_size = 10;
    // ps.c_space_batch_size = 1;  // robot is not redundant
    ps.min_valid_samples = 50;
    ps.max_iters = 50;

    // ps.sampler_type = SamplerType::GRID;
    ps.tsr_resolution = { 1, 1, 1, 1, 1, 30 };

    // solve it!
    auto solution = solve(regions, joint_limits, f_ik, f_is_valid, f_path_cost, f_state_cost, ps);

    showPath(solution.path, rviz, robot);

    return 0;
}
