#include <ros/ros.h>

#include <Eigen/Core>

#include <moveit_msgs/GetCartesianPath.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <simple_moveit_wrapper/industrial_robot.h>

#include <ocpl_tsr/task_space_regions.h>
#include <ocpl_planning/acro_planner.h>
#include <ocpl_planning/cost_functions.h>
#include <ocpl_ros/io.h>

namespace smw = simple_moveit_wrapper;

class PlanningServer
{
    ros::ServiceServer plan_service_;
    ros::NodeHandle node_handle_;

    // visualization
    moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

    // ocpl interface
    std::shared_ptr<smw::Robot> robot_;
    std::shared_ptr<ocpl::Planner> planner_;
    ocpl::PlannerSettings planner_settings_;

  public:
    /********************
     * Setup
     ********************/
    PlanningServer(const std::string& base_link_name = "world")
    {
        plan_service_ = node_handle_.advertiseService("ocpl_get_cartesian_path", &PlanningServer::callback, this);

        // initialize MoveIt visual tools
        visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(base_link_name, "/visualization_marker_array"));
        visual_tools_->loadMarkerPub(true);
        visual_tools_->loadRobotStatePub("/display_robot_state", true);
        ros::Duration(0.2).sleep();  // delay to make sure al the messages got where they had to be

        // initialize simple moveit wrapper
        robot_ = std::make_shared<smw::IndustrialRobot>("manipulator", "tool_tip");

        // Translate robot kinematics to solver interface
        ocpl::JointLimits joint_limits;
        for (auto bound : robot_->getJointPositionLimits())
        {
            joint_limits.push_back(ocpl::Bounds{ bound.lower, bound.upper });
            ROS_INFO_STREAM("Joint limits: " << bound.lower << ", " << bound.upper);
        }

        ocpl::Robot bot{ robot_->getNumDof(),
                         robot_->getNumRedDof(),
                         joint_limits,
                         [this](const ocpl::JointPositions& q) { return robot_->fk(q); },
                         [this](const ocpl::Transform& tf, const ocpl::JointPositions& /*q_fixed*/) {
                             return robot_->ik(tf);
                         },
                         [this](const ocpl::JointPositions& q) { return !robot_->isColliding(q); } };

        std::string PLANNER_SETTINGS_FILE{ "kuka/kuka_gpq.yaml" };
        planner_settings_ = ocpl::loadSettingsFromFile(PLANNER_SETTINGS_FILE);

        planner_ = std::make_shared<ocpl::UnifiedPlanner>(bot, planner_settings_);

        ROS_INFO("Ready to receive planning requests.");
    }

    bool callback(moveit_msgs::GetCartesianPath::Request& req, moveit_msgs::GetCartesianPath::Response& res)
    {
        ROS_INFO_STREAM("OCPL planning server received planning request.\n" << req);
        clearRvizMarkers();
        plotWaypoints(req.waypoints);

        auto task = msgToTask(req);

        auto latest_settings = ocpl::loadSettingsFromFile("kuka/kuka_gpq.yaml");
        planner_->changeSettings(latest_settings);

        // make sure recently published collision objects are known
        robot_->updatePlanningScene();

        auto solution = planner_->solve(task, ocpl::L2NormDiff2, ocpl::zeroStateCost);

        robot_->animatePath(visual_tools_, solution.path);

        res.fraction = (double) solution.path.size() / (double) task.size();
        res.solution.joint_trajectory.points.clear();
        double time{0.0}, time_step{0.1};
        for (auto qi : solution.path)
        {
            trajectory_msgs::JointTrajectoryPoint pt;
            pt.positions = qi;
            pt.time_from_start = ros::Duration(time);
            time += time_step;
            res.solution.joint_trajectory.points.push_back(pt);
        }

        return true;
    }
    /********************
     * The Planner
     ********************/
    std::vector<ocpl::TSR> msgToTask(const moveit_msgs::GetCartesianPath::Request& req)
    {
        // only parse orientation constraints for now
        // assume they are the tolerances represent a symmetric interval around the nominal orientation
        // that's the only thing the orientation constraint message supports

        ocpl::TSRBounds bounds{ { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 } };
        if (!req.path_constraints.orientation_constraints.empty())
        {
            double x_tol = req.path_constraints.orientation_constraints.at(0).absolute_x_axis_tolerance;
            double y_tol = req.path_constraints.orientation_constraints.at(0).absolute_y_axis_tolerance;
            double z_tol = req.path_constraints.orientation_constraints.at(0).absolute_z_axis_tolerance;
            bounds.rx = { -x_tol / 2.0, x_tol / 2.0 };
            bounds.ry = { -y_tol / 2.0, y_tol / 2.0 };
            bounds.rz = { -z_tol / 2.0, z_tol / 2.0 };
        }

        for (auto bound : bounds.asVector())
        {
            ROS_INFO_STREAM("Tolerance: " << bound.lower << ", " << bound.upper);
        }

        std::vector<ocpl::TSR> task;
        for (auto wp : req.waypoints)
        {
            Eigen::Isometry3d tf =
                Eigen::Translation3d(wp.position.x, wp.position.y, wp.position.z) *
                Eigen::Quaterniond(wp.orientation.w, wp.orientation.x, wp.orientation.y, wp.orientation.z);

            task.push_back(ocpl::TSR{ tf, bounds });
        }
        return task;
    }

    /********************
     * Visualization
     ********************/
    void plotWaypoints(const std::vector<geometry_msgs::Pose>& waypoints)
    {
        for (auto wp : waypoints)
        {
            visual_tools_->publishAxis(wp);
        }
        visual_tools_->trigger();
        ros::Duration(0.2).sleep();
    }

    void clearRvizMarkers()
    {
        visual_tools_->deleteAllMarkers();
        visual_tools_->trigger();
        ros::Duration(0.2).sleep();
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ocpl_cart_planning_server");

    PlanningServer server;

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
