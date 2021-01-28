#pragma once

#include <string>
#include <vector>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
// #include <moveit/collision_distance_field/collision_robot_hybrid.h>
// #include <moveit/collision_distance_field/collision_world_hybrid.h>

#include <ocpl_ros/threadsafe_state_storage.h>
#include <ocpl_tsr/task_space_regions.h>  // for the Bounds type

#include <Eigen/Dense>

namespace ocpl
{
typedef Eigen::Isometry3d Transform;
typedef std::vector<double> JointPositions;
typedef std::vector<std::vector<double>> IKSolution;

/** \brief Wrapper around MoveIt to simplify some basic operations.**/
class MoveItRobot
{
  protected:
    robot_model::RobotModelPtr kinematic_model_;
    // robot_state::RobotStatePtr kinematic_state_;
    const robot_state::JointModelGroup* joint_model_group_;

    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;

    planning_scene::PlanningScenePtr planning_scene_;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

    bool check_collisions_ = true;

    std::string tcp_frame_;
    std::size_t num_dof_;

    std::vector<Bounds> joint_position_limits_;
    std::vector<Bounds> joint_velocity_limits_;

    mutable TSStateStorage state_storage_;

    // const collision_detection::CollisionWorldHybrid* hy_world_;
    // const collision_detection::CollisionRobotHybrid* hy_robot_;

    // void initDistanceFields();
    // void testDistanceFields();

  public:
    MoveItRobot(const std::string& tcp_frame = "tool0");
    virtual ~MoveItRobot() = default;

    const Transform& fk(const JointPositions& q) const;
    const Transform& fk(const JointPositions& q, const std::string& frame) const;

    /** \brief Inverse kinematics
     *
     * This function should be overriden with a robot specific implementation of analytical inverse kinematics.
     * The default class uses MoveIts default inverse kinematics plugin (which could have analytical inverse kinematics.
     *
     * (TODO) Return multiple solutions from MoveIts IK plugin.
     *
     * **/
    virtual IKSolution ik(const Transform& tf);

    /** \brief Redundant inverse kinematics
     *
     * The robot's joints are divided in two groups. The first k joints are called "redundant joints",
     * and the last 3 / 6 joints are called "base joints".
     * Given as input positions for the redundant joints, an analytical inverse kinematics solver can calculate
     * the base joints for the given end-effector pose.
     *
     * **/
    virtual IKSolution ik(const Transform& tf, const std::vector<double>& q_redundant);
    Eigen::MatrixXd jacobian(const JointPositions& q);

    const Transform& getLinkFixedRelativeTransform(const std::string& name) const;
    void updatePlanningScene();
    bool isInCollision(const JointPositions& joint_pose) const;
    bool isPathColliding(const JointPositions& q_from, const JointPositions& q_to, int steps) const;
    void plot(moveit_visual_tools::MoveItVisualToolsPtr mvt, JointPositions& joint_pose,
              const rviz_visual_tools::colors& color = rviz_visual_tools::DEFAULT);
    std::size_t getNumDof()
    {
        return num_dof_;
    }

    const std::vector<Bounds>& getJointPositionLimits()
    {
        return joint_position_limits_;
    }

    const std::vector<Bounds>& getJointVelocityLimits()
    {
        return joint_velocity_limits_;
    }
};

}  // namespace ocpl
