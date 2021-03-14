#pragma once

#include <ocpl_ros/planning_cases/util.h>

namespace l_profile
{
using Eigen::AngleAxisd;
using Eigen::Vector3d;

std::vector<ocpl::TSR> waypoints(double x_offset = 0.98, bool extra_tolerance = false)
{
    ocpl::TSRBounds bounds{ { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { -M_PI, M_PI } };

    // replace bounds if required to avoid a specific obstacle for the welding torch
    if (extra_tolerance)
    {
        bounds = ocpl::TSRBounds{ { 0, 0 }, { 0, 0 }, { 0, 0 }, { -0.5, 0.5 }, { 0, 0 }, { -M_PI, M_PI } };
    }

    Eigen::Isometry3d ori(AngleAxisd(0.0, Vector3d::UnitX()) * AngleAxisd(deg2rad(135), Vector3d::UnitY()) *
                          AngleAxisd(deg2rad(90), Vector3d::UnitZ()));
    Eigen::Vector3d start(x_offset, -0.5, 0.02);
    Eigen::Vector3d stop(x_offset, 0.5, 0.02);
    const std::size_t num_points{ 30 };
    // const double total_distance = (stop - start).norm();
    return createLineTask(bounds, start, stop, ori, num_points);
}
}  // namespace l_profile
