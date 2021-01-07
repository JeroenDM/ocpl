#include <ros/ros.h>

#include <algorithm>
#include <limits>
#include <chrono>

#include <ocpl_ros/moveit_robot_examples.h>
#include <ocpl_sampling/grid_sampler.h>
#include <ocpl_sampling/halton_sampler.h>
#include <ocpl_sampling/random_sampler.h>
#include <ocpl_tsr/task_space_regions.h>

#include <ocpl_planning/planners.h>
#include <ocpl_planning/factories.h>
#include <ocpl_planning/cost_functions.h>

#include <ocpl_benchmark/benchmark.h>

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

/** \brief Demo for a planar, 6 link robot.
 *
 * You can specify and load collision objects with the python script "load_collision_objects.py".
 * The main function shows how to setup and solve a task of following a simple straight line.
 *
 * */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "ocpl_benchmark1");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    PlanarRobotNR robot;

    //////////////////////////////////
    // Create task
    //////////////////////////////////
    // 2P 3R robot case
    // auto regions = createCase1();
    // JointLimits joint_limits{ { 2.0, 3.0 }, { 0.0, 0.9 } };  // joint limits for the redundant joints

    // small passage case
    auto regions = createCase2();
    JointLimits joint_limits{ { -1.5, 1.5 }, { -1.5, 1.5 }, { -1.5, 1.5 } };  // for redundant joints

    // 8 dof zig zag case
    // auto regions = createCase3();
    // JointLimits joint_limits{};
    // for (int i = 0; i < 5; ++i)
    //     joint_limits.push_back({ -1.5, 1.5 });

    //////////////////////////////////
    // Simple interface solver
    //////////////////////////////////
    Problem pb;
    pb.redundant_joint_limits = joint_limits;

    // function that tells you whether a state is valid (collision free)
    pb.is_valid_fun = [&robot](const JointPositions& q) { return !robot.isInCollision(q); };

    // function that returns analytical inverse kinematics solution for end-effector pose
    pb.ik_fun = [&robot](const Transform& tf, const JointPositions& q_fixed) { return robot.ik(tf, q_fixed); };

    // specify an objective to minimize the cost along the path
    // here we use a predefined function that uses the L1 norm of the different
    // between two joint positions
    pb.path_cost_fun = L1NormDiff2;

    // optionally we can set a state cost for every point along the path
    // this one tries to keep the end-effector pose close the the nominal pose
    // defined in the task space region
    // auto f_state_cost = [&robot](const TSR& tsr, const JointPositions& q) {
    //     return poseDistance(tsr.tf_nominal, robot.fk(q)).norm();
    // };
    pb.state_cost_fun = zeroStateCost;

    // settings to select a planner
    // PlannerSettings ps;
    // ps.name = "grid_30";
    // ps.is_redundant = true;
    // ps.sampler_type = SamplerType::HALTON;
    // ps.t_space_batch_size = 10;
    // ps.c_space_batch_size = 100;
    // ps.min_valid_samples = 50;
    // ps.max_iters = 200;

    // // ps.sampler_type = SamplerType::GRID;
    // ps.tsr_resolution = { 1, 1, 1, 1, 1, 30 };
    // ps.redundant_joints_resolution = std::vector<int>(robot.getNumDof(), 6);

    // case 2
    PlannerSettings grid_2;
    grid_2.is_redundant = true;
    grid_2.name = "grid_2";
    grid_2.sampler_type = SamplerType::GRID;
    grid_2.tsr_resolution = { 1, 1, 1, 1, 1, 30 };
    grid_2.redundant_joints_resolution = { 6, 6, 6 };

    PlannerSettings halton_2;
    halton_2.is_redundant = true;
    halton_2.name = "halton_2";
    halton_2.sampler_type = SamplerType::HALTON;
    halton_2.t_space_batch_size = 30;
    halton_2.c_space_batch_size = 6 * 6 * 6;
    // For the first table in the paper I only did a single run
    halton_2.max_iters = 1;

    PlannerSettings random_2;
    random_2.is_redundant = true;
    random_2.name = "random_2";
    random_2.sampler_type = SamplerType::RANDOM;
    random_2.t_space_batch_size = 30;
    random_2.c_space_batch_size = 6 * 6 * 6;
    // For the first table in the paper I only did a single run
    random_2.max_iters = 1;

    runBenchmark("benchmark_case_2.csv", pb, regions, { grid_2, halton_2, random_2 }, 20);

    return 0;
}
