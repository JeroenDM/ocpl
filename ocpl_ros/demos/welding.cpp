#include <ros/ros.h>

#include <algorithm>

#include <ocpl_ros/moveit_robot_examples.h>
#include <ocpl_ros/rviz.h>
#include <ocpl_ros/io.h>

#include <ocpl_sampling/grid_sampler.h>
#include <ocpl_sampling/halton_sampler.h>
#include <ocpl_sampling/random_sampler.h>
#include <ocpl_tsr/task_space_regions.h>

#include <ocpl_planning/acro_planner.h>
#include <ocpl_planning/factories.h>
#include <ocpl_planning/cost_functions.h>
#include <ocpl_planning/types.h>

#include <ocpl_ros/planning_cases/puzzle.h>
#include <ocpl_ros/planning_cases/teapot.h>
#include <ocpl_ros/planning_cases/l_profile.h>

// ROS Trajectory Action server definition
#include <control_msgs/FollowJointTrajectoryAction.h>
// Means by which we communicate with above action-server
#include <actionlib/client/simple_action_client.h>

using namespace ocpl;

void showPath(const std::vector<JointPositions>& path, Rviz& rviz, MoveItRobot& robot, double dt = 0.5)
{
    for (JointPositions q : path)
    {
        if (robot.isInCollision(q))
            robot.plot(rviz.visual_tools_, q, rviz_visual_tools::RED);
        else
            // robot.plot(rviz.visual_tools_, q, rviz_visual_tools::GREEN);
            robot.plot(rviz.visual_tools_, q, rviz_visual_tools::DEFAULT);
        ros::Duration(dt).sleep();
    }
}

int main(int argc, char** argv)
{
    using Eigen::AngleAxisd;
    using Eigen::Vector3d;

    ros::init(argc, argv, "ocpl_moveit_demo");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    IndustrialRobot robot("tool_tip");
    // IndustrialRobot robot("tool0");
    Rviz rviz("base_link");
    rviz.clear();

    std::vector<double> q_start = { 1.0, -1.0, 1.0, -1.0, 1.0, -1.0 };
    std::vector<double> q_zero(robot.getNumDof(), 0.0);
    // std::vector<double> q_start = { 0.0, 0.0, 0.0 };
    auto tf_start = robot.fk(q_start);
    auto q_start_ik = robot.ik(tf_start);

    // std::cout << tf_start.translation().transpose() << std::endl;

    // rviz.plotPose(tf_start);
    // robot.plot(rviz.visual_tools_, q_start);
    // ros::Duration(0.5).sleep();

    // std::cout << "Found " << q_start_ik.size() << " ik solutions.\n";
    // for (auto q : q_start_ik)
    // {
    //     robot.plot(rviz.visual_tools_, q);
    //     ros::Duration(0.5).sleep();
    // }

    //////////////////////////////////
    // l_profile
    //////////////////////////////////
    auto task = l_profile::waypoints(/* x_offset */ 0.98, /* extra_tolerance */ true);
    for (auto tsr : task)
    {
        rviz.plotPose(tsr.tf_nominal);
        ros::Duration(0.1).sleep();
    }
    ros::Duration(0.5).sleep();

    //////////////////////////////////
    // glass of water
    //////////////////////////////////
    // auto task = teapot::waypoints();
    // auto bounds = teapot::tsrBounds();

    // EigenSTL::vector_Vector3d visual_path;
    // int skipper{ 0 };
    // for (auto tsr : task)
    // {
    //     if (skipper % 3 == 0)
    //     {
    //         rviz.plotPose(tsr.tf_nominal);
    //         visual_path.push_back(tsr.tf_nominal.translation());
    //         ros::Duration(0.1).sleep();
    //     }
    //     skipper++;
    // }
    // ros::Duration(0.5).sleep();
    // auto f_path_cost = L2NormDiff2;

    //////////////////////////////////
    // Puzzle
    //////////////////////////////////
    // auto welding_task = puzzle::waypoints();
    // auto bounds = puzzle::tsrBounds();

    // // select only the first 50 points
    // std::vector<TSR> task;
    // task.reserve(100);
    // std::copy_n(welding_task.begin(), 100, std::back_inserter(task));

    // EigenSTL::vector_Vector3d path_positions;
    // for (auto& pt : task)
    // {
    //     path_positions.push_back(pt.tf_nominal.translation());
    // }

    // rviz.visual_tools_->publishPath(path_positions);
    // ros::Duration(1.0).sleep();

    // // define a path cost that puts an upper limit on the joint motion between two waypoints
    // // this limit is calculated from the desired end-effector speed along the path and the robot's joint velocity
    // limits
    // auto dq_max = calcMaxJointStep(task, robot.getJointVelocityLimits(), /* end-effector speed */ 0.05);
    // auto f_path_cost = [&dq_max](const std::vector<double>& n1, const std::vector<double>& n2) {
    //     assert(n1.size() == n2.size());
    //     assert(dq_max.size() == n1.size());

    //     double cost{ 0.0 };
    //     for (int i = 0; i < n1.size(); ++i)
    //     {
    //         double inc = std::abs(n1[i] - n2[i]);
    //         if (inc > dq_max[i])
    //             return std::nan("1");
    //         cost += inc;
    //     }
    //     // if (cost > 1.0)
    //     //     cost = std::numeric_limits<double>::max();
    //     return cost;
    // };

    //////////////////////////////////
    // Simple interface solver
    //////////////////////////////////
    // joint limits for the redundant joints
    JointLimits joint_limits{};

    // function that tells you whether a state is valid (collision free)
    auto f_is_valid = [&robot](const JointPositions& q) { return !robot.isInCollision(q); };
    // auto f_is_valid = [&robot](const JointPositions& q) { return true; };

    // function that returns analytical inverse kinematics solution for end-effector pose
    auto f_ik = [&robot](const Transform& tf, const JointPositions& /*q_fixed*/) { return robot.ik(tf); };

    Robot bot{ robot.getNumDof(),
               robot.getNumDof() - 3,
               robot.getJointPositionLimits(),
               [&robot](const JointPositions& q) { return robot.fk(q); },
               f_ik,
               f_is_valid };

    //////////////////////////////////
    // Planning settings
    //////////////////////////////////
    // PlannerSettings ps;
    // ps.is_redundant = true;
    // ps.sampler_type = SamplerType::HALTON;
    // // ps.sampler_type = SamplerType::RANDOM;
    // ps.t_space_batch_size = 10;
    // ps.c_space_batch_size = 100;
    // ps.min_valid_samples = 100;
    // ps.max_iters = 500;
    PlannerSettings ps = loadSettingsFromFile("kuka1.yaml");

    auto f_path_cost = [&ps](const std::vector<double>& n1, const std::vector<double>& n2) {
        assert(n1.size() == n2.size());
        double cost{ 0.0 };
        for (int i = 0; i < n1.size(); ++i)
        {
            double inc = std::abs(n1[i] - n2[i]);
            if (inc > ps.cspace_delta)
                return std::nan("1");
            cost += inc * inc;
        }
        return cost;
    };

    // optionally we can set a state cost for every point along the path
    // this one tries to keep the end-effector pose close the the nominal pose
    // defined in the task space region
    auto f_state_cost = [&robot, &ps](const TSR& tsr, const JointPositions& q) {
        Eigen::VectorXd dv = poseDistance(tsr.tf_nominal, robot.fk(q));
        // only penalize the x and y rotation
        return ps.state_cost_weight * std::sqrt(dv[3] * dv[3] + dv[4] * dv[4]);
    };
    // auto f_state_cost = zeroStateCost;

    // ps.sampler_type = SamplerType::GRID;
    // ps.tsr_resolution = { 1, 1, 1, 1, 1, 10 };

    // keep close to robot home pose
    std::vector<double> q_home{ 0, -1.5708, 1.5708, 0, 0, 0 };
    // robot.plot(rviz.visual_tools_, q_home, rviz_visual_tools::MAGENTA);
    // ros::Duration(0.5).sleep();

    // auto f_state_cost = [q_home](const TSR& tsr, const JointPositions& q) { return L2NormDiff2(q, q_home); };

    //////////////////////////////////
    // Solve it
    //////////////////////////////////
    UnifiedPlanner planner(bot, ps);

    // // solve it!
    auto solution = planner.solve(task, f_path_cost, f_state_cost);

    robot.animatePath(rviz.visual_tools_, solution.path);

    return 0;
}
