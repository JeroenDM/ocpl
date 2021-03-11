#include <ros/ros.h>

#include <algorithm>
#include <limits>
#include <chrono>

#include <ocpl_ros/moveit_robot_examples.h>
#include <ocpl_ros/rviz.h>
#include <ocpl_ros/io.h>
#include <ocpl_ros/planning_cases/case1.h>
#include <ocpl_ros/planning_cases/case2.h>
#include <ocpl_ros/planning_cases/case3.h>

#include <ocpl_sampling/grid_sampler.h>
#include <ocpl_sampling/halton_sampler.h>
#include <ocpl_sampling/random_sampler.h>
#include <ocpl_tsr/task_space_regions.h>

#include <ocpl_planning/acro_planner.h>
#include <ocpl_planning/oriolo.h>
#include <ocpl_planning/factories.h>
#include <ocpl_planning/cost_functions.h>

#include <ocpl_benchmark/benchmark.h>

using namespace ocpl;

/** \brief Demo for a planar, 6 link robot.
 *
 * You can specify and load collision objects with the python script "load_collision_objects.py".
 * The main function shows how to setup and solve a task of following a simple straight line.
 *
 * */
int main(int argc, char** argv)
{
    // ros specific setup
    ros::init(argc, argv, "ocpl_moveit_demo_rn");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Simplified visualization interface
    Rviz rviz;
    rviz.clear();

    // Select the correct type of robot
    // All robots in this script are assumed to be planar
    PlanarRobotNR robot;

    //////////////////////////////////
    // Select the planning task
    // And get the case specific parameters
    //////////////////////////////////
    int PLANNING_CASE{ 2 };
    if (argc > 1)
    {
        PLANNING_CASE = std::stoi(argv[1]);
        ROS_INFO_STREAM("Selected planning case " << PLANNING_CASE);
    }

    std::vector<TSR> regions;
    CaseSettings case_settings;
    switch (PLANNING_CASE)
    {
        case 1:
            // 2P 3R robot case
            regions = case1::waypoints();
            case_settings = case1::settings();
            // alternative joint limits for this case for the 2p 3r robot
            // JointLimits joint_limits{ { 2.0, 3.0 }, { -2.0, 1.0 }, { -1.5, 1.5 }, { -1.5, 1.5 }, { -1.5, 1.5 } };
            break;
        case 2:
            // 6 dof small passage case
            regions = case2::waypoints(20);
            case_settings = case2::settings();
            break;

        case 3:
            // 8 dof zig zag case
            regions = case3::waypoints();
            case_settings = case3::settings();
            break;

        default:
            ROS_WARN("Unkown planning case id.");
            return 0;
    }
    std::vector<Bounds> tsr_bounds = regions.at(0).bounds.asVector();  // use first waypoint bounds for all waypoints

    // visualize the end-effector path
    for (TSR& tsr : regions)
    {
        rviz.plotPose(tsr.tf_nominal);
        ros::Duration(0.05).sleep();
    }

    //////////////////////////////////
    // Setup the planner settings
    //////////////////////////////////
    // function that tells you whether a state is valid (collision free)
    auto is_valid_fun = [&robot](const JointPositions& q) { return !robot.isInCollision(q); };

    // function that returns analytical inverse kinematics solution for end-effector pose
    auto ik_fun = [&robot](const Transform& tf, const JointPositions& q_fixed) { return robot.ik(tf, q_fixed); };

    // optionally we can set a state cost for every point along the path
    // this one tries to keep the end-effector pose close the the nominal pose
    // defined in the task space region
    auto state_cost_fun = [&robot](const TSR& tsr, const JointPositions& q) {
        return poseDistance(tsr.tf_nominal, robot.fk(q)).norm();
    };
    // auto state_cost_fun = zeroStateCost;

    Robot bot{ robot.getNumDof(),
               robot.getNumRedDof(),
               robot.getJointPositionLimits(),
               [&robot](const JointPositions& q) { return robot.fk(q); },
               ik_fun,
               is_valid_fun };

    for (auto jl : robot.getJointPositionLimits())
    {
        std::cout << "Joint limit: " << jl.lower << ", " << jl.upper << "\n";
    }

    auto ps = loadSettingsFromFile("local1.yaml");
    ps.tsr_resolution = case_settings.tsr_resolution;
    ps.redundant_joints_resolution = case_settings.redundant_joints_resolution;
    // double max_l2_norm_diff = std::sqrt((double)robot.getNumDof() * ps.cspace_delta * ps.cspace_delta);

    // // auto f_path_cost = L2NormDiff2;
    // auto f_path_cost = [max_l2_norm_diff](const std::vector<double>& n1, const std::vector<double>& n2) {
    //     assert(n1.size() == n2.size());
    //     double cost{ 0.0 };
    //     for (int i = 0; i < n1.size(); ++i)
    //     {
    //         double inc = std::abs(n1[i] - n2[i]);
    //         if (inc > max_l2_norm_diff)
    //             return std::nan("1");
    //         cost += inc * inc;
    //     }
    //     return cost;
    // };

    //////////////////////////////////
    // Solve the problem
    //////////////////////////////////
    UnifiedPlanner planner(bot, ps);
    // std::reverse(regions.begin(), regions.end());
    // Solution solution = planner.solve(regions, f_path_cost, state_cost_fun);
    Solution solution = planner.solve(regions);

    if (solution.success)
    {
        std::cout << "A solution is found with a cost of " << solution.cost << "\n";
    }
    else
    {
        std::cout << "No complete solution was found.\n";
    }

    robot.animatePath(rviz.visual_tools_, solution.path);

    //////////////////////////////////
    // Benchmark
    //////////////////////////////////
    // std::vector<std::string> file_names = readLinesFromFile("benchmark1_file_names.txt");
    // std::vector<PlannerSettings> settings;
    // ROS_INFO("Running benchmark for the settings files:");
    // for (auto name : file_names)
    // {
    //     ROS_INFO_STREAM(name);
    //     settings.push_back(loadSettingsFromFile(name));

    //     // change case and robot specific settings
    //     settings.back().tsr_resolution = case_settings.tsr_resolution;
    //     settings.back().redundant_joints_resolution = case_settings.redundant_joints_resolution;
    // }

    // UnifiedPlanner planner(bot, settings.front());
    // std::string outfilename{ "results/benchmark_1_case_" + std::to_string(PLANNING_CASE) + ".csv" };
    // runBenchmark(outfilename, bot, regions, planner, settings, 5);

    // outfilename = "results/benchmark_1_case_" + std::to_string(PLANNING_CASE) + "_rev.csv";
    // std::reverse(regions.begin(), regions.end());
    // runBenchmark(outfilename, bot, regions, planner, settings, 5);

    return 0;
}
