#pragma once

#include <string>
#include <vector>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <Eigen/Dense>

namespace ocpl
{
typedef Eigen::Isometry3d Transform;
typedef std::vector<double> JointPositions;
typedef std::vector<std::vector<double>> IKSolution;

/** \brief Simple interpolation between two vectors. Returns one vector.**/
template <typename T>
inline std::vector<T> interpolate(std::vector<T> q_from, std::vector<T> q_to, T s)
{
    std::vector<T> q(q_from.size());
    for (std::size_t i = 0; i < q_from.size(); ++i)
    {
        q[i] = (1 - s) * q_from[i] + s * q_to[i];
    }
    return q;
}

/** \brief Wrapper around MoveIt to simplify some basic operations.**/
class MoveItRobot
{
  protected:
    robot_model::RobotModelPtr kinematic_model_;
    robot_state::RobotStatePtr kinematic_state_;
    const robot_state::JointModelGroup* joint_model_group_;

    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;

    planning_scene::PlanningScenePtr planning_scene_;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

    bool check_collisions_ = true;

    std::string tcp_frame_;
    std::size_t ndof_;

  public:
    MoveItRobot(const std::string& tcp_frame = "tool0");
    ~MoveItRobot() = default;

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
};

}  // namespace ocpl
