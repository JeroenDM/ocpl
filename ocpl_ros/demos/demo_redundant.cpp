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

/** Case 1 from my 2018 paper. **/
std::vector<TSR> createCase1()
{
    Transform tf1 = Transform::Identity();
    tf1.translation() << 0.5, 2.0, 0.0;
    TSRBounds bounds{ { -0.2, 0.3 }, { 0.0, 0.0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { -M_PI, 0.0 } };
    std::vector<TSR> task;

    Transform tf = tf1;
    double step = (2.5 - 2.0) / 5.0;
    for (int i{ 0 }; i < 5; ++i)
    {
        tf.translation() += step * Eigen::Vector3d::UnitY();
        task.push_back({ tf, bounds });
    }
    return task;
}

/** Case 2 from my 2018 paper. **/
std::vector<TSR> createCase2()
{
    const double small_passage_width{ 0.5 };
    Transform tf1 = Transform::Identity();
    tf1.translation() << 4.0, small_passage_width / 2, 0.0;
    TSRBounds bounds{ { 0.0, 0.0 }, { 0.0, 0.0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, M_PI } };
    std::vector<TSR> task;

    Transform tf = tf1;
    for (int i{ 0 }; i < 5; ++i)
    {
        tf.translation() += 0.2 * Eigen::Vector3d::UnitX();
        task.push_back({ tf, bounds });
    }
    return task;
}

/** Case 3 from my 2018 paper. **/
std::vector<TSR> createCase3()
{
    Transform tf1 = Transform::Identity();
    tf1.translation() << 5.0, 1.3, 0.0;
    TSRBounds bounds{ { 0.0, 0.0 }, { 0.0, 0.0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { -M_PI_2, M_PI_2 } };
    std::vector<TSR> task;

    Transform tf = tf1;
    double step = (2.5 - 1.3) / 5.0;
    for (int i{ 0 }; i < 5; ++i)
    {
        tf.translation() += step * Eigen::Vector3d::UnitY();
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
        ros::Duration(0.5).sleep();
    }
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

    PlanarRobotNR robot;
    Rviz rviz;
    ros::Duration(0.2).sleep();
    rviz.clear();
    ros::Duration(0.2).sleep();

    // create an interesting start configurations
    std::vector<double> q_start(robot.getNumDof(), 1.0);

    auto tf_start = robot.fk(q_start);
    JointPositions q_fixed(q_start.begin() + 3, q_start.end());
    auto solution = robot.ik(tf_start, q_fixed);

    std::cout << tf_start.translation().transpose() << std::endl;

    rviz.plotPose(tf_start);
    robot.plot(rviz.visual_tools_, q_start);

    ROS_INFO_STREAM("Found " << solution.size() << " ik solutions.");
    for (auto q : solution)
    {
        robot.plot(rviz.visual_tools_, q, rviz_visual_tools::GREEN);
        ros::Duration(0.5).sleep();
    }

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
    // 2P 3R robot case
    auto regions = createCase1();
    JointLimits joint_limits{{2.0, 3.0}, {0.0, 0.9}};  // joint limits for the redundant joints

    // small passage case
    // auto regions = createCase2();
    // JointLimits joint_limits{ { -1.5, 1.5 }, { -1.5, 1.5 }, { -1.5, 1.5 } };  // joint limits for the redundant joints

    // 8 dof zig zag case
    // auto regions = createCase3();
    // JointLimits joint_limits{};
    // for (int i = 0; i < 5; ++i)
    //     joint_limits.push_back({ -1.5, 1.5 });

    for (TSR& tsr : regions)
    {
        rviz.plotPose(tsr.tf_nominal);
        ros::Duration(0.05).sleep();
    }

    //////////////////////////////////
    // Simple interface solver
    //////////////////////////////////

    // function that tells you whether a state is valid (collision free)
    auto f_is_valid = [&robot](const JointPositions& q) { return !robot.isInCollision(q); };

    // function that returns analytical inverse kinematics solution for end-effector pose
    auto f_ik = [&robot](const Transform& tf, const JointPositions& q_fixed) { return robot.ik(tf, q_fixed); };

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
    ps.is_redundant = true;
    // ps.sampler_type = SamplerType::HALTON;
    ps.t_space_batch_size = 10;
    ps.c_space_batch_size = 100;
    ps.min_valid_samples = 50;
    ps.max_iters = 200;

    ps.sampler_type = SamplerType::GRID;
    // ps.tsr_resolution = { 1, 1, 1, 1, 1, 20 };
    // ps.redundant_joints_resolution = std::vector<int>(robot.getNumDof(), 5);

    // case 1
    ps.tsr_resolution = { 5, 1, 1, 1, 1, 32 };
    ps.redundant_joints_resolution = {10, 9};

    // solve it!
    auto path = solve(regions, joint_limits, f_ik, f_is_valid, f_path_cost, f_state_cost, ps);

    showPath(path, rviz, robot);
    return 0;
}
