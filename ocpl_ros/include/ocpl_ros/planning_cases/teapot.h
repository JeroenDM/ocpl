/** Pour some tea with a robot
 * 
 * roslaunch setup_teapot_moveit_config demo.launch
 * rosrun ocpl_ros load_collision_objects.py teapot
 * 
 * **/
#pragma once

#include <ocpl_ros/planning_cases/util.h>

namespace teapot
{
std::vector<ocpl::TSR> waypoints(size_t num_points = 30)
{
    using Eigen::AngleAxisd;
    using Eigen::Vector3d;

    ocpl::TSRBounds bounds{ { 0.0, 0.0 }, { 0, 0 }, { -0.05, 0.05 }, { -1.3, 1.3 }, { 0, 0 }, { 0.0, 0.0 } };

    Eigen::Isometry3d ori(AngleAxisd(deg2rad(90), Vector3d::UnitY()) * AngleAxisd(deg2rad(-90), Vector3d::UnitX()));
    Eigen::Vector3d start(0.98, 0.0, 0.8);

    ocpl::Transform tf(ori);
    tf.translation() = start;

    const double total_distance = 1.1;
    double step = total_distance / num_points;

    std::vector<ocpl::TSR> task;
    for (int i{ 0 }; i < num_points; ++i)
    {
        tf = tf * Eigen::AngleAxisd(step, Eigen::Vector3d::UnitY());
        task.push_back(ocpl::TSR{ tf, bounds });
        task.back().local_ = false;
    }
    return task;
}

std::vector<ocpl::Bounds> tsrBounds()
{
    return { { 0.0, 0.0 }, { 0, 0 }, { -0.05, 0.05 }, { -1.3, 1.3 }, { 0, 0 }, { 0.0, 0.0 } };
}
}  // namespace teapot
