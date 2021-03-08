#include <ros/ros.h>

#include <algorithm>
#include <limits>
#include <chrono>

#include <ocpl_ros/moveit_robot_examples.h>
#include <ocpl_ros/rviz.h>
#include <ocpl_ros/io.h>
#include <ocpl_ros/planning_cases/halfopen_box.h>

#include <ocpl_tsr/task_space_regions.h>

#include <ocpl_planning/acro_planner.h>
#include <ocpl_planning/oriolo.h>
#include <ocpl_planning/factories.h>
#include <ocpl_planning/cost_functions.h>

using namespace ocpl;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ocpl_moveit_demo_rn");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    IndustrialRobot robot("tool_tip");
    Rviz rviz;
    rviz.clear();

    //////////////////////////////////
    // Create task
    //////////////////////////////////
    auto task = halfopen_box::waypoints();

    EigenSTL::vector_Vector3d visual_path;
    for (TSR& tsr : task)
    {
        rviz.plotPose(tsr.tf_nominal);
        ros::Duration(0.05).sleep();
        visual_path.push_back(tsr.tf_nominal.translation());
    }
    // rviz.visual_tools_->publishPath(visual_path);
    // rviz.visual_tools_->trigger();

    //////////////////////////////////
    // Simple interface solver
    //////////////////////////////////
    // function that tells you whether a state is valid (collision free)
    auto is_valid_fun = [&robot](const JointPositions& q) { return !robot.isInCollision(q); };

    // function that returns analytical inverse kinematics solution for end-effector pose
    auto ik_fun = [&robot](const Transform& tf, const JointPositions& q_fixed) { return robot.ik(tf, q_fixed); };

    Robot bot{ robot.getNumDof(),
               robot.getNumDof() - 6,
               robot.getJointPositionLimits(),
               [&robot](const JointPositions& q) { return robot.fk(q); },
               ik_fun,
               is_valid_fun };

    PlannerSettings ps = loadSettingsFromFile("halfopen_box1.yaml");

    // specify an objective to minimize the cost along the path
    // here we use a predefined function that uses the L1 norm of the different
    // between two joint positions
    // auto path_cost_fun = L1NormDiff2;

    // // try to limit joint speed
    // this should actually  not be part of a path cost, but a hard constraint
    // but it can be combined into a single function because they potentially use the same calculation
    // std::vector<double> max_joint_speed(robot.getNumDof(), 1.5);

    auto path_cost_fun = [&ps](const std::vector<double>& n1, const std::vector<double>& n2) {
        assert(n1.size() == n2.size());
        // assert(max_joint_speed.size() == n1.size());

        double cost{ 0.0 };
        for (int i = 0; i < n1.size(); ++i)
        {
            double inc = std::abs(n1[i] - n2[i]);
            inc = std::min(inc, 2 * M_PI - inc);
            if (inc > ps.cspace_delta)
                return std::nan("1");
            cost += inc;
        }
        return cost;
    };
    // auto path_cost_fun = L2NormDiff2;

    // optionally we can set a state cost for every point along the path
    // this one tries to keep the end-effector pose close the the nominal pose
    // defined in the task space region
    auto state_cost_fun = [&robot](const TSR& tsr, const JointPositions& q) {
        return poseDistance(tsr.tf_nominal, robot.fk(q)).norm();
    };
    // auto state_cost_fun = zeroStateCost;

    //////////////////////////////////
    // solve it!
    //////////////////////////////////
    // auto orioli_ps = loadOrioloSettings("oriolo1.txt");
    oriolo::OrioloPlanner planner("oriolo", bot, ps);
    // 
    // UnifiedPlanner planner(bot, ps);
    Solution res = planner.solve(task, path_cost_fun, state_cost_fun);
    if (res.success)
    {
        std::cout << "A solution is found with a cost of " << res.cost << "\n";
    }
    else
    {
        std::cout << "No complete solution was found.\n";
    }

    robot.animatePath(rviz.visual_tools_, res.path);

    return 0;
}
