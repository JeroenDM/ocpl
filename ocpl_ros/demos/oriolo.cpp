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
#include <ocpl_ros/planning_cases/puzzle.h>

#include <ocpl_sampling/grid_sampler.h>
#include <ocpl_sampling/halton_sampler.h>
#include <ocpl_sampling/random_sampler.h>
#include <ocpl_tsr/task_space_regions.h>

#include <ocpl_planning/cost_functions.h>
#include <ocpl_planning/oriolo.h>
#include <ocpl_planning/planner2.h>
#include <ocpl_planning/io.h>

#include <ocpl_benchmark/benchmark.h>

using namespace ocpl;

oriolo::OrioloSpecificSettings loadOrioloSettings(const std::string filename)
{
    auto map = readSettingsFile(filename);

    oriolo::OrioloSpecificSettings settings;
    settings.METHOD = findOrDefault(map, "method", settings.METHOD);
    settings.D = findOrDefault(map, "d", settings.D);
    settings.MAX_SHOTS = findOrDefault(map, "max_shots", settings.MAX_SHOTS);
    settings.MAX_ITER = findOrDefault(map, "max_iter", settings.MAX_ITER);
    settings.MAX_EXTEND = findOrDefault(map, "max_extend", settings.MAX_EXTEND);
    return settings;
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

    // std::shared_ptr<MoveItRobot> robot = std::make_shared<PlanarRobotNR>();
    std::shared_ptr<MoveItRobot> robot = std::make_shared<IndustrialRobot>();

    Rviz rviz;
    ros::Duration(0.2).sleep();
    rviz.clear();
    ros::Duration(0.2).sleep();

    //////////////////////////////////
    // Create task
    //////////////////////////////////
    // 2P 3R robot case
    // auto regions = case1::waypoints();
    // auto tsr_bounds = case1::tsrBounds();
    // alternative joint limits for this case for the 2p 3r robot
    // JointLimits joint_limits{ { 2.0, 3.0 }, { -2.0, 1.0 }, { -1.5, 1.5 }, { -1.5, 1.5 }, { -1.5, 1.5 } };

    // small passage case
    // auto regions = case2::waypoints(20);
    // auto tsr_bounds = case2::tsrBounds();

    // 8 dof zig zag case
    // auto regions = case3::waypoints();
    // auto tsr_bounds = case3::tsrBounds();

    // puzzle piece from ROS-Industrial descartes package tutorials
    auto regions = puzzle::waypoints();
    auto tsr_bounds = puzzle::tsrBounds();

    EigenSTL::vector_Vector3d path_pos;
    for (TSR& tsr : regions)
    {
        rviz.plotPose(tsr.tf_nominal);
        // ros::Duration(0.05).sleep();
        path_pos.push_back(tsr.tf_nominal.translation());
    }
    rviz.visual_tools_->publishPath(path_pos, rviz_visual_tools::RED, 0.1);
    ros::Duration(0.1).sleep();

    //////////////////////////////////
    // Describe the robot
    //////////////////////////////////
    Robot bot{ robot->getNumDof(),
               robot->getNumDof() - 3,
               robot->getJointPositionLimits(),
               [&robot](const JointPositions& q) { return robot->fk(q); },
               [&robot](const ocpl::Transform& tf, const JointPositions& q_red) { return robot->ik(tf, q_red); },
               [&robot](const JointPositions& q) { return !robot->isInCollision(q); } };

    for (auto jl : robot->getJointPositionLimits())
    {
        std::cout << "Joint limits: " << jl.lower << ", " << jl.upper << "\n";
    }

    //////////////////////////////////
    // Try Oriolo algorithms
    //////////////////////////////////
    // oriolo::OrioloSpecificSettings ps;
    // ps.METHOD = "bigreedy";
    // // // ps.METHOD = "quispe";
    // // ps.MAX_SHOTS = 2000;
    // ps.MAX_ITER = 2000;
    auto ps = loadOrioloSettings("oriolo1.txt");
    oriolo::OrioloPlanner planner("oriolo", bot, ps);
    // // std::reverse(regions.begin(), regions.end());

    auto solution = planner.solve(regions);

    if (solution.path.empty())
    {
        std::cout << "No solution found.\n";
    }
    {
        std::cout << "Found solution of length: " << solution.path.size() << "\n";
        std::cout << "Path cost: " << solution.cost << "\n";
        robot->animatePath(rviz.visual_tools_, solution.path);
        // savePath("latest_path.npy", solution.path);
    }

    //////////////////////////////////
    // Some quick benchmarking
    //////////////////////////////////
    // oriolo::OrioloSpecificSettings set1;
    // set1.METHOD = "greedy";
    // set1.MAX_ITER = 2000;
    // set1.name = set1.METHOD;

    // oriolo::OrioloSpecificSettings set2;
    // set2.METHOD = "bigreedy";
    // set2.MAX_ITER = 2000;
    // set2.name = set2.METHOD;

    // runBenchmark("oriolo_red_1.csv", bot, regions, { set1, set2 }, 50);

    // std::reverse(regions.begin(), regions.end());
    // runBenchmark("oriolo_red_1_reversed.csv", bot, regions, { set1, set2 }, 50);

    // Planner2Settings set1{};
    // set1.name = "set1";
    // set1.method = "local_stack";

    // Planner2Settings set2{};
    // set2.name = "set2";
    // set2.method = "local_priority_stack";

    // runBenchmark("planner2_1_ptr.csv", bot, regions, { set1, set2 }, 50);

    // std::reverse(regions.begin(), regions.end());
    // runBenchmark("oriolo_red_1_reversed.csv", bot, regions, { set1, set2 }, 50);

    return 0;
}
