#pragma once

#include <ocpl_ros/planning_cases/util.h>

namespace case2
{
/** Case 2 from my 2018 paper. **/
std::vector<ocpl::TSR> waypoints(int num_points = 5)
{
    ocpl::TSRBounds bounds{ { 0.0, 0.0 }, { 0.0, 0.0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { -M_PI, M_PI } };
    Eigen::Vector3d start(4.0, 0.25, 0.0);
    Eigen::Vector3d stop(5.0, 0.25, 0.0);
    return createLineTask(bounds, start, stop, Eigen::Isometry3d::Identity(), num_points);
}

ocpl::CaseSettings settings()
{
    ocpl::CaseSettings s;
    s.tsr_resolution = {1, 1, 1, 1, 1, 64};
    s.redundant_joints_resolution = {32, 32, 32};
    return s;
}

}  // namespace case2
