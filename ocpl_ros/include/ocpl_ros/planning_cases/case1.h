#pragma once

#include <ocpl_ros/planning_cases/util.h>

namespace case1
{
/** Case 1 modification**/
std::vector<ocpl::TSR> waypoints()
{
    ocpl::TSRBounds bounds{ { 0.0, 0.0 }, { 0.0, 0.0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { -M_PI, M_PI } };
    Eigen::Vector3d start(0.5, 2.0, 0.0);
    Eigen::Vector3d stop(0.5, 2.7, 0.0);
    auto line1 = createLineTask(bounds, start, stop, Eigen::Isometry3d::Identity(), 10);

    Eigen::Vector3d start2(0.5, 2.7, 0.0);
    Eigen::Vector3d stop2(2.5, 2.7, 0.0);
    auto line2 = createLineTask(bounds, start2, stop2, Eigen::Isometry3d::Identity(), 30);

    Eigen::Vector3d start3(2.5, 2.7, 0.0);
    Eigen::Vector3d stop3(2.5, 2.0, 0.0);
    auto line3 = createLineTask(bounds, start3, stop3, Eigen::Isometry3d::Identity(), 10);

    line1.insert(line1.end(), line2.begin(), line2.end());
    line1.insert(line1.end(), line3.begin(), line3.end());

    // std::reverse(line1.begin(), line1.end());

    return line1;
}

ocpl::CaseSettings settings()
{
    ocpl::CaseSettings s;
    s.tsr_resolution = {1, 1, 1, 1, 1, 64};
    s.redundant_joints_resolution = {40, 40};
    return s;
}
}  // namespace case1
