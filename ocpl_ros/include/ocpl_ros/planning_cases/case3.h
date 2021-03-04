#pragma once

#include <ocpl_ros/planning_cases/util.h>

namespace case3
{
/** Case 3 from my 2018 paper. **/
std::vector<ocpl::TSR> waypoints()
{
    ocpl::TSRBounds bounds{ { 0.0, 0.0 }, { 0.0, 0.0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { -M_PI_2, M_PI_2 } };
    Eigen::Vector3d start(5.0, 1.3, 0.0);
    Eigen::Vector3d stop(5.0, 2.5, 0.0);
    return createLineTask(bounds, start, stop, Eigen::Isometry3d::Identity(), 10);
}

std::vector<ocpl::Bounds> tsrBounds()
{
    return { { 0.0, 0.0 }, { 0.0, 0.0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, M_PI } };
}
}  // namespace case3
