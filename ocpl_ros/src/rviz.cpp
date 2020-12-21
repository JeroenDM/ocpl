#include <ocpl_ros/rviz.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

namespace rvt = rviz_visual_tools;

namespace ocpl
{
Rviz::Rviz()
{
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("world", "/visualization_marker_array"));
    visual_tools_->loadMarkerPub(true);
    visual_tools_->loadRobotStatePub("/display_robot_state", true);
}

void Rviz::plotPose(const Transform& pose)
{
    visual_tools_->publishAxis(pose, rvt::LARGE);
    visual_tools_->trigger();
}

void Rviz::clear()
{
    visual_tools_->deleteAllMarkers();
    visual_tools_->trigger();
}
}  // namespace ocpl
