#include <ros/ros.h>

#include <algorithm>
#include <limits>
#include <chrono>

#include <simple_moveit_wrapper/industrial_robot.h>
#include <ocpl_ros/rviz.h>
#include <ocpl_ros/io.h>
#include <ocpl_ros/planning_cases/halfopen_box.h>
#include <ocpl_ros/planning_cases/text.h>

#include <ocpl_tsr/task_space_regions.h>

#include <ocpl_planning/acro_planner.h>
#include <ocpl_planning/oriolo.h>
#include <ocpl_planning/factories.h>
#include <ocpl_planning/cost_functions.h>
#include <ocpl_planning/io.h>

using namespace ocpl;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ocpl_moveit_demo_rn");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    simple_moveit_wrapper::IndustrialRobot robot("manipulator", "tool_tip");
    Rviz rviz;
    rviz.clear();

    simple_moveit_wrapper::Robot table("rotation_table", "work");
    auto work_pose = table.fk({ 0.0 });
    std::cout << "Work position: " << work_pose.translation().transpose() << "\n";

    //////////////////////////////////
    // Create task
    //////////////////////////////////
    // auto task = halfopen_box::waypoints();
    // auto task = text::waypoints(work_pose);
    auto task = text::waypoints(Eigen::Isometry3d::Identity());

    EigenSTL::vector_Vector3d visual_path;
    for (TSR& tsr : task)
    {
        // rviz.plotPose(tsr.tf_nominal);
        // ros::Duration(0.05).sleep();
        visual_path.push_back(tsr.tf_nominal.translation());
    }
    rviz.visual_tools_->publishPath(visual_path);
    rviz.visual_tools_->trigger();
    ros::Duration(0.2).sleep();

    //////////////////////////////////
    // Define solver interface functions
    //////////////////////////////////
    auto is_valid_fun = [&robot](const JointPositions& q) { return !robot.isColliding(q); };

    auto fk_fun = [&robot, &table](const JointPositions& q) {
        // take of the first joint value for the rotation table
        std::vector<double> qr(q.begin() + 1, q.end());

        // express the fk pose in the reference frame of the rotation table
        return table.fk({ q[0] }).inverse() * robot.fk(qr);
    };

    auto ik_fun = [&robot, &table](const Transform& tf, const JointPositions& q_fixed) {
        assert(q_fixed.size() == 2);
        auto tf_ik = table.fk({ q_fixed[0] }) * tf;
        auto ik_sols = robot.ik(tf_ik, { q_fixed[1] });

        for (std::vector<double>& sol : ik_sols)
        {
            sol.insert(sol.begin(), q_fixed[0]);
        }
        return ik_sols;
    };

    std::vector<ocpl::Bounds> joint_limits;
    auto jl_smw = robot.getJointPositionLimits();
    for (auto l : jl_smw)
    {
        joint_limits.push_back(ocpl::Bounds{ l.lower, l.upper });
    }
    joint_limits.insert(joint_limits.begin(), ocpl::Bounds{ -0.7, 0.7 });

    Robot bot{ robot.getNumDof() + table.getNumDof(),
               robot.getNumDof() - 6 + table.getNumDof(),
               joint_limits,
               fk_fun,
               ik_fun,
               is_valid_fun };

    PlannerSettings ps = loadSettingsFromFile("halfopen_box1.yaml");

    auto path_cost_fun = L2NormDiff2;

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
    // oriolo::OrioloPlanner planner(bot, ps);

    UnifiedPlanner planner(bot, ps);
    // std::reverse(task.begin(), task.end());
    // Solution res = planner.solve(task);
    Solution res = planner.solve(task, path_cost_fun, zeroStateCost);
    if (res.success)
    {
        std::cout << "A solution is found with a cost of " << res.cost << "\n";
    }
    else
    {
        std::cout << "No complete solution was found.\n";
    }

    // robot.animatePath(rviz.visual_tools_, res.path);

    // savePath("last_path.npy", res.path);

    //

    // // analyse path
    // for (size_t wp{0}; wp < res.path.size(); ++wp)
    // {
    //     auto tf_fk = robot.fk(res.path[wp]);
    //     auto v = task[wp].poseToValues(tf_fk);
    //     std::cout << "v: " << v << std::endl;
    //

    // split robot path
    std::vector<JointPositions> table_path;
    std::vector<JointPositions> robot_path;
    for (auto qi : res.path)
    {
        table_path.push_back({ qi[0] });
        robot_path.push_back(std::vector<double>(qi.begin() + 1, qi.end()));
    }

    // animate both paths at the same time
    for (std::size_t wp{ 0 }; wp < robot_path.size(); ++wp)
    {
        // robot.plot(rviz.visual_tools_, robot_path.at(wp));
        // table.plot(rviz.visual_tools_, table_path.at(wp));
        // robot.plot(rviz.visual_tools_, res.path.at(wp));
        robot.plotMultipleGroups(rviz.visual_tools_,
                                 { { "manipulator", robot_path.at(wp) }, { "rotation_table", table_path.at(wp) } });
        ros::Duration(0.1).sleep();
    }

    savePath("last_path.npy", res.path);

    // for (auto qi : table_path)
    // {
    //     std::cout << qi[0] << ", ";
    // }
    // std::cout << "\n";

    // EigenSTL::vector_Vector3d ee_pos;
    // for (auto qi : robot_path)
    // {
    //     ee_pos.push_back(robot.fk(qi).translation());
    // }
    // rviz.visual_tools_->publishPath(ee_pos);
    // rviz.visual_tools_->trigger();
    // ros::Duration(0.2).sleep();

    return 0;
}
