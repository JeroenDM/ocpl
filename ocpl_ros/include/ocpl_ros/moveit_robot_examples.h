#pragma once

#include <ocpl_ros/moveit_robot.h>

namespace ocpl
{
class PlanarRobot3R : public MoveItRobot
{
  public:
    IKSolution ik(const Transform& tf) override;
};

class PlanarRobotNR : public MoveItRobot
{
    /** \brief Planar robots have 3 base joints for analytical inverse kinematics. **/
    const std::size_t num_base_joints_{ 3 };
    std::string analytical_ik_base_link_;
    std::vector<double> analytical_ik_link_length_;

    void messyHardCodedStuff();

  public:
    PlanarRobotNR(const std::string& tcp_frame = "tool0");
    IKSolution ik(const Transform& tf) override;
    IKSolution ik(const Transform& tf, const std::vector<double>& q_redundant) override;
};

}  // namespace ocpl
