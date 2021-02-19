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

#include <ocpl_planning/cost_functions.h>
#include <ocpl_planning/oriolo.h>
#include <ocpl_planning/planner2.h>
#include <ocpl_planning/io.h>

#include <ocpl_benchmark/benchmark.h>

using namespace ocpl;

std::ostream& operator<<(std::ostream& os, const JointPositions& q)
{
    for (auto qi : q)
        os << qi << ", ";
    return os;
}

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

/** Case 1 modification**/
std::vector<TSR> createCase1()
{
    TSRBounds bounds{ { 0.0, 0.0 }, { 0.0, 0.0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { -M_PI, 0.0 } };
    Eigen::Vector3d start(0.5, 2.0, 0.0);
    Eigen::Vector3d stop(0.5, 2.7, 0.0);
    auto line1 = createLineTask(bounds, start, stop, Eigen::Isometry3d::Identity(), 10);

    Eigen::Vector3d start2(0.5, 2.7, 0.0);
    Eigen::Vector3d stop2(2.5, 2.7, 0.0);
    auto line2 = createLineTask(bounds, start2, stop2, Eigen::Isometry3d::Identity(), 30);

    Eigen::Vector3d start3(2.5, 2.7, 0.0);
    Eigen::Vector3d stop3(2.5, 2.0, 0.0);
    auto line3 = createLineTask(bounds, start3, stop3, Eigen::Isometry3d::Identity(), 10);

    line1.insert(line1.end(), line2.begin(), line2.end());
    line1.insert(line1.end(), line3.begin(), line3.end());

    // std::reverse(line1.begin(), line1.end());

    return line1;
}

/** Case 2 from my 2018 paper. **/
std::vector<TSR> createCase2(int num_points = 5)
{
    TSRBounds bounds{ { 0.0, 0.0 }, { 0.0, 0.0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, M_PI } };
    Eigen::Vector3d start(4.0, 0.25, 0.0);
    Eigen::Vector3d stop(5.0, 0.25, 0.0);
    return createLineTask(bounds, start, stop, Eigen::Isometry3d::Identity(), num_points);
}

/** Case 3 from my 2018 paper. **/
std::vector<TSR> createCase3()
{
    TSRBounds bounds{ { 0.0, 0.0 }, { 0.0, 0.0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { -M_PI_2, M_PI_2 } };
    Eigen::Vector3d start(5.0, 1.3, 0.0);
    Eigen::Vector3d stop(5.0, 2.5, 0.0);
    return createLineTask(bounds, start, stop, Eigen::Isometry3d::Identity(), 10);
}

void showPath(const std::vector<JointPositions>& path, Rviz& rviz, std::shared_ptr<MoveItRobot> robot)
{
    for (JointPositions q : path)
    {
        if (robot->isInCollision(q))
            robot->plot(rviz.visual_tools_, q, rviz_visual_tools::RED);
        else
            robot->plot(rviz.visual_tools_, q, rviz_visual_tools::GREEN);
        ros::Duration(0.1).sleep();
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

    std::shared_ptr<MoveItRobot> robot = std::make_shared<PlanarRobotNR>();
    Rviz rviz;
    ros::Duration(0.2).sleep();
    rviz.clear();
    ros::Duration(0.2).sleep();

    //////////////////////////////////
    // Create task
    //////////////////////////////////
    // 2P 3R robot case
    auto regions = createCase1();
    JointLimits joint_limits{ { 2.0, 3.0 }, { -2.0, 1.0 }, { -1.5, 1.5 }, { -1.5, 1.5 }, { -1.5, 1.5 } };
    std::vector<Bounds> tsr_bounds{ { 0.0, 0.0 }, { 0.0, 0.0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { -M_PI, 0.0 } };

    // small passage case
    // auto regions = createCase2(30);
    // // JointLimits joint_limits{ { -1.5, 1.5 }, { -1.5, 1.5 }, { -1.5, 1.5 } };  // for redundant joints
    // std::vector<Bounds> joint_limits{ { -1.5, 1.5 }, { -1.5, 1.5 }, { -1.5, 1.5 },
    //                                   { -1.5, 1.5 }, { -1.5, 1.5 }, { -1.5, 1.5 } };  // for all joints
    // std::vector<Bounds> tsr_bounds{ { 0.0, 0.0 }, { 0.0, 0.0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, M_PI } };

    // 8 dof zig zag case
    // auto regions = createCase3();
    // JointLimits joint_limits{};
    // for (int i = 0; i < robot->getNumDof(); ++i)
    //     joint_limits.push_back({ -1.5, 1.5 });
    // std::vector<Bounds> tsr_bounds{ { 0.0, 0.0 }, { 0.0, 0.0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, M_PI } };

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
    // Create another task
    //////////////////////////////////
    // std::vector<TSR> task2;
    // {
    //     Eigen::Vector3d start(4.0, 0.25, 0.0);
    //     TSRBounds bounds{ { 0.0, 0.0 }, { 0.0, 0.0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 } };
    //     Transform tf = Eigen::Isometry3d::Identity();
    //     tf.translation() = start;

    //     size_t num_points = 30;

    //     double step = 1.2 / num_points;
    //     for (int i{ 0 }; i < num_points; ++i)
    //     {
    //         tf = tf * Eigen::AngleAxisd(step, Eigen::Vector3d::UnitZ());
    //         task2.push_back(TSR{ tf, bounds });
    //     }
    // }
    // std::vector<Bounds> tsr_bounds_2 = task2.front().bounds.asVector();
    // for (auto tsr : task2)
    //     rviz.plotPose(tsr.tf_nominal);

    //////////////////////////////////
    // Describe the robot
    //////////////////////////////////
    Robot bot{ robot->getNumDof(),
               robot->getNumDof() - 3,
               joint_limits,
               [&robot](const JointPositions& q) { return robot->fk(q); },
               [&robot](const ocpl::Transform& tf, const JointPositions& q_red) { return robot->ik(tf, q_red); },
               [&robot](const JointPositions& q) { return !robot->isInCollision(q); } };

    //////////////////////////////////
    // Try Oriolo algorithms
    //////////////////////////////////
    // oriolo::OrioloSpecificSettings ps;
    // ps.METHOD = "bigreedy";
    // // ps.METHOD = "quispe";
    // // ps.MAX_SHOTS = 2000;
    // ps.MAX_ITER = 2000;
    // oriolo::OrioloPlanner planner("oriolo", bot, ps);
    // // oriolo::Planner planner(fkFun, ikFun, isValidFun, joint_limits, tsr_bounds, robot->getNumDof(),
    // //                         robot->getNumDof() - 3);

    // // oriolo::Planner planner(fkFun, ikFun, isValidFun, joint_limits, tsr_bounds, robot->getNumDof(),
    // //                         robot->getNumDof() - 3);

    // // planner.setTask(regions);

    // // std::reverse(regions.begin(), regions.end());

    // auto solution = planner.solve(regions);
    // // auto solution = planner.rrtLike(regions);
    // // auto solution = planner.greedy2();

    //////////////////////////////////
    // Try new generic algorithms
    //////////////////////////////////
    Planner2Settings settings{};
    settings.method = "local_priority_stack";
    Planner2 planner("planner2", bot, settings);

    Solution solution = planner.solve(regions);

    if (solution.path.empty())
    {
        std::cout << "No solution found.\n";
    }
    {
        std::cout << "Found solution of length: " << solution.path.size() << "\n";
        std::cout << "Path cost: " << solution.cost << "\n";
        showPath(solution.path, rviz, robot);
        // savePath("latest_path.npy", solution.path);
    }

    // auto solution = planner.step(0, 2, q1, regions);
    // std::cout << "step() sol size " << solution.size() << "\n";
    // //std::cout << "step() " << solution[0] << "\n";

    // solution = planner.greedy(regions);
    // std::cout << "greedy() sol size " << solution.size() << "\n";

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
