/** Rviz wrapper
 *
 * Collect code I often use to visualize stuff in rviz.
 * Not really super extra useful compared to the existing visual tools class,
 * but I like it this way.
 *
 * */
#pragma once

#include <moveit_visual_tools/moveit_visual_tools.h>

namespace ocpl
{
typedef Eigen::Isometry3d Transform;

struct Rviz
{
    Rviz(const std::string& base_link_name = "world");

    /** \brief The visual tools ptr is public so others can visualize things freely using the same instance. **/
    moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

    /** \brief visualize a 3D pose in rviz using red-green-blue axes. **/
    void plotPose(const Transform& pose);

    /** \brief Delete all markers and pull the trigger. **/
    void clear();
};
}  // namespace ocpl
