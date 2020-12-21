#pragma once

#include <ocpl_ros/moveit_robot.h>

namespace ocpl
{
class PlanarRobot3R : public MoveItRobot
{
  public:
    IKSolution ik(const Transform& tf) override;
};

}  // namespace ocpl
