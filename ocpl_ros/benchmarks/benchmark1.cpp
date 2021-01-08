#include <ros/ros.h>

#include <algorithm>
#include <limits>
#include <chrono>
#include <fstream>

#include <ocpl_ros/moveit_robot_examples.h>
#include <ocpl_sampling/grid_sampler.h>
#include <ocpl_sampling/halton_sampler.h>
#include <ocpl_sampling/random_sampler.h>
#include <ocpl_tsr/task_space_regions.h>

#include <ocpl_planning/everything.h>

#include <ocpl_benchmark/benchmark.h>

using namespace ocpl;

/** This script runs the 3 planar robot cases of my 2018 paper again.
 *
 * Provide the case number and the number of repeats as command line arguments.
 * For example, to run case 2 50 times:
 *      rosrun ocpl_ros benchmark1 2 50
 *
 * But first load the moveit configurations for the robot, and the collision objects.
 *
 *      roslaunch XXX_moveit_config demo.launch
 *      rosrun ocpl_ros load_collision_objects.py XXX
 *
 * All tasks are straight lines, hence the task creation helper function createLineTask.
 *
 * Planner settings are saved to case_X_settings.txt.
 * Results are saved to case_X_results.csv.
 *
 * **/

std::vector<TSR> createLineTask(TSRBounds bounds, Eigen::Vector3d start, Eigen::Vector3d stop,
                                Eigen::Isometry3d orientation, std::size_t num_points)
{
    std::vector<TSR> task;
    Transform tf(orientation);
    tf.translation() = start;

    Eigen::Vector3d direction = (stop - start).normalized();

    double step = (stop - start).norm() / num_points;
    for (int i{ 0 }; i < 5; ++i)
    {
        tf.translation() += step * direction;
        task.push_back({ tf, bounds });
    }
    return task;
}

/** Case 1 from my 2018 paper. **/
std::vector<TSR> createCase1()
{
    TSRBounds bounds{ { -0.2, 0.3 }, { 0.0, 0.0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { -M_PI, 0.0 } };
    Eigen::Vector3d start(0.5, 2.0, 0.0);
    Eigen::Vector3d stop(0.5, 2.5, 0.0);
    return createLineTask(bounds, start, stop, Eigen::Isometry3d::Identity(), 5);
}

/** Case 2 from my 2018 paper. **/
std::vector<TSR> createCase2()
{
    TSRBounds bounds{ { 0.0, 0.0 }, { 0.0, 0.0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, M_PI } };
    Eigen::Vector3d start(4.0, 0.25, 0.0);
    Eigen::Vector3d stop(5.0, 0.25, 0.0);
    return createLineTask(bounds, start, stop, Eigen::Isometry3d::Identity(), 5);
}

/** Case 3 from my 2018 paper. **/
std::vector<TSR> createCase3()
{
    TSRBounds bounds{ { 0.0, 0.0 }, { 0.0, 0.0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { -M_PI_2, M_PI_2 } };
    Eigen::Vector3d start(5.0, 1.3, 0.0);
    Eigen::Vector3d stop(5.0, 2.5, 0.0);
    return createLineTask(bounds, start, stop, Eigen::Isometry3d::Identity(), 5);
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

    //////////////////////////////////
    // Parse command line input
    //////////////////////////////////

    int case_id{ 1 };
    int num_repeats{ 1 };
    if (argc > 1)
    {
        case_id = std::stoi(std::string(argv[1]));
    }
    if (argc > 2)
    {
        num_repeats = std::stoi(std::string(argv[2]));
    }
    ROS_INFO_STREAM("Running case " << case_id << " " << num_repeats << " times.");

    //////////////////////////////////
    // Planner settings
    //////////////////////////////////
    PlannerSettings grid_2;
    grid_2.is_redundant = true;
    grid_2.name = "grid_2";
    grid_2.sampler_type = SamplerType::GRID;

    PlannerSettings halton_2;
    halton_2.is_redundant = true;
    halton_2.name = "halton_2";
    halton_2.sampler_type = SamplerType::HALTON;
    // For the first table in the paper I only did a single run
    halton_2.max_iters = 1;

    PlannerSettings random_2;
    random_2.is_redundant = true;
    random_2.name = "random_2";
    random_2.sampler_type = SamplerType::RANDOM;
    // For the first table in the paper I only did a single run
    random_2.max_iters = 1;

    // modified below depending on the case

    //////////////////////////////////
    // Create task
    //////////////////////////////////
    PlanarRobotNR robot;

    std::vector<TSR> task;
    JointLimits joint_limits;

    switch (case_id)
    {
        case 1:
            // 2P 3R robot case
            task = createCase1();
            joint_limits = { { 2.0, 3.0 }, { 0.0, 0.9 } };  // joint limits for the redundant joints
            grid_2.tsr_resolution = { 5, 1, 1, 1, 1, 32 };
            grid_2.redundant_joints_resolution = { 10, 9 };
            halton_2.t_space_batch_size = 32;
            halton_2.c_space_batch_size = 10 * 9;
            break;
        case 2:

            // small passage case
            task = createCase2();
            joint_limits = { { -1.5, 1.5 }, { -1.5, 1.5 }, { -1.5, 1.5 } };  // for redundant joints
            grid_2.tsr_resolution = { 1, 1, 1, 1, 1, 30 };
            grid_2.redundant_joints_resolution = { 6, 6, 6 };
            halton_2.t_space_batch_size = 30;
            halton_2.c_space_batch_size = 6 * 6 * 6;
            break;
        case 3:
            // 8 dof zig zag case
            task = createCase3();
            joint_limits = {};
            for (int i = 0; i < 5; ++i)
                joint_limits.push_back({ -1.5, 1.5 });
            grid_2.tsr_resolution = { 1, 1, 1, 1, 1, 20 };
            grid_2.redundant_joints_resolution = { 5, 5, 5, 5, 5 };
            halton_2.t_space_batch_size = 20;
            halton_2.c_space_batch_size = 5 * 5 * 5 * 5 * 5;
            break;

        default:
            ROS_ERROR_STREAM("Unknowns case id: " << case_id);
            exit(1);
    }

    // random and halton incremental sampling use the same settings
    random_2.t_space_batch_size = halton_2.t_space_batch_size;
    random_2.c_space_batch_size = halton_2.c_space_batch_size;

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

    std::string filename {"case_"};
    filename.append(std::to_string(case_id));
    filename.append("_");

    std::ofstream file(filename + "settings.txt");
    if (file.is_open())
    {
        file << grid_2 << halton_2 << random_2;
        file.close();
    }

    runBenchmark(filename + "results.csv", pb, task, { grid_2, halton_2, random_2 }, num_repeats);

    return 0;
}
