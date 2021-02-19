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
#include <ocpl_planning/jdm.h>

using namespace ocpl;

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

/** Case 1 from my 2018 paper. **/
std::vector<TSR> createCase1()
{
    TSRBounds bounds{ { -0.2, 0.3 }, { 0.0, 0.0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { -M_PI, 0.0 } };
    Eigen::Vector3d start(0.5, 2.0, 0.0);
    Eigen::Vector3d stop(0.5, 2.5, 0.0);
    return createLineTask(bounds, start, stop, Eigen::Isometry3d::Identity(), 5);
}

/** Case 1 modification**/
std::vector<TSR> createCase1bis()
{
    TSRBounds bounds{ { 0.0, 0.0 }, { 0.0, 0.0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { -M_PI, M_PI } };
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
    return createLineTask(bounds, start, stop, Eigen::Isometry3d::Identity(), 5);
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
    auto ik_solution = robot.ik(tf_start, q_fixed);

    std::cout << tf_start.translation().transpose() << std::endl;

    rviz.plotPose(tf_start);
    robot.plot(rviz.visual_tools_, q_start);

    ROS_INFO_STREAM("Found " << ik_solution.size() << " ik solutions.");
    for (auto q : ik_solution)
    {
        robot.plot(rviz.visual_tools_, q, rviz_visual_tools::DEFAULT);
        ros::Duration(0.2).sleep();
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
    auto regions = createCase1bis();
    // JointLimits joint_limits{ { 2.0, 3.0 }, { 0.0, 0.9 } };  // joint limits for the redundant joints
    JointLimits joint_limits{ { 2.0, 3.0 }, { -2.0, 1.0 } };  // joint limits for the redundant joints

    // small passage case
    // auto regions = createCase2(20);
    // JointLimits joint_limits{ { -1.5, 1.5 }, { -1.5, 1.5 }, { -1.5, 1.5 } };  // for redundant joints

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
    auto state_cost_fun = [&robot](const TSR& tsr, const JointPositions& q) {
        return poseDistance(tsr.tf_nominal, robot.fk(q)).norm();
    };
    // auto state_cost_fun = zeroStateCost;

    Robot bot{ robot.getNumDof(),
               robot.getNumDof() - 3,
               joint_limits,
               [&robot](const JointPositions& q) { return robot.fk(q); },
               ik_fun,
               is_valid_fun };

    // settings to select a planner
    PlannerSettings ps;
    ps.is_redundant = true;
    // ps.sampler_type = SamplerType::HALTON;
    // ps.t_space_batch_size = 10;
    // ps.c_space_batch_size = 100;
    // ps.min_valid_samples = 50;
    // ps.max_iters = 200;

    ps.sampler_type = SamplerType::GRID;
    // ps.tsr_resolution = { 1, 1, 1, 1, 1, 30 };
    // ps.redundant_joints_resolution = std::vector<int>(redundant_joint_limits.size(), 6);

    //////////////////////////////////
    // Planners
    //////////////////////////////////
    // case 1
    // ps.tsr_resolution = { 5, 1, 1, 1, 1, 32 };
    // ps.redundant_joints_resolution = { 10, 9 };
    ps.tsr_resolution = { 1, 1, 1, 1, 1, 100 };
    ps.redundant_joints_resolution = { 10, 50 };
    // ps.redundant_joints_resolution = { 6, 6, 6 };

    // solve it!
    // Solution solution = solve(regions, joint_limits, ik_fun, is_valid_fun, path_cost_fun, state_cost_fun, ps);

    jdm::JdmPlanner planner("grid_search", bot, ps);
    Solution solution = planner.solve(regions);

    if (solution.success)
    {
        std::cout << "A solution is found with a cost of " << solution.cost << "\n";
    }
    else
    {
        std::cout << "No complete solution was found.\n";
    }

    showPath(solution.path, rviz, robot);

    return 0;
}
