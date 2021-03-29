#pragma once

#include <ocpl_ros/planning_cases/util.h>

namespace halfopen_box
{
using Eigen::AngleAxisd;
using Eigen::Vector3d;
using Eigen::Isometry3d;
std::vector<ocpl::TSR> waypoints()
{
    double da = deg2rad(20);
    ocpl::TSRBounds bounds{ { 0, 0 }, { 0, 0 }, { 0, 0 }, { -da, da }, { -da, da }, { -M_PI, M_PI } };
    // ocpl::TSRBounds bounds{ { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { -M_PI, M_PI } };
    Vector3d work(2, 0.3, 0.9027);
    Vector3d start(0.0, 1.05, 0.05);
    Vector3d stop(0.9, 1.05, 0.05);
    Isometry3d ori(AngleAxisd(deg2rad(135), Vector3d::UnitX()) * AngleAxisd(0.0, Vector3d::UnitY()) *
                   AngleAxisd(0.0, Vector3d::UnitZ()));
    auto task = createLineTask(bounds, start + work, stop + work, ori, 20);

    // add another line
    Vector3d start_2(0.95, 1.1, 0.05);
    Vector3d stop_2(0.95, 2.9, 0.05);
    Isometry3d ori_2(AngleAxisd(0.0, Vector3d::UnitX()) * AngleAxisd(deg2rad(135), Vector3d::UnitY()) *
                     AngleAxisd(deg2rad(90), Vector3d::UnitZ()));
    auto second_line = createLineTask(bounds, start_2 + work, stop_2 + work, ori_2, 40);
    // auto task = second_line;

    // and add another line
    Vector3d start_3(0.9, 2.95, 0.05);
    Vector3d stop_3(0.0, 2.95, 0.05);
    Isometry3d ori_3(AngleAxisd(deg2rad(-135), Vector3d::UnitX()) * AngleAxisd(0.0, Vector3d::UnitY()) *
                     AngleAxisd(deg2rad(180), Vector3d::UnitZ()));
    auto third_line = createLineTask(bounds, start_3 + work, stop_3 + work, ori_3, 20);

    // connect them with a circle segment
    ocpl::Transform tf_a(ori);
    tf_a.translation() = stop + work;
    Vector3d centre(0.9, 1.1, 0.05);
    centre += work;
    auto circle_segment = createCircleSegment(bounds, tf_a, start_2 + work, centre, Eigen::Vector3d::UnitZ(), 10);

    ocpl::Transform tf_a_2(ori_2);
    tf_a_2.translation() = stop_2 + work;
    Vector3d centre_2(0.9, 2.9, 0.05);
    centre_2 += work;
    auto circle_segment_2 = createCircleSegment(bounds, tf_a_2, start_3 + work, centre_2, Eigen::Vector3d::UnitZ(),
    10);

    task.insert(task.end(), circle_segment.begin(), circle_segment.end());
    task.insert(task.end(), second_line.begin(), second_line.end());
    task.insert(task.end(), circle_segment_2.begin(), circle_segment_2.end());
    task.insert(task.end(), third_line.begin(), third_line.end());
    return task;
}
}  // namespace halfopen_box
