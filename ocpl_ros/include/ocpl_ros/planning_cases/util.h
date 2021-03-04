#pragma once

#include <vector>

#include <Eigen/Core>

#include <ocpl_planning/types.h>
#include <ocpl_tsr/task_space_regions.h>

std::vector<ocpl::TSR> createLineTask(ocpl::TSRBounds bounds, Eigen::Vector3d start, Eigen::Vector3d stop,
                                Eigen::Isometry3d orientation, std::size_t num_points)
{
    std::vector<ocpl::TSR> task;
    ocpl::Transform tf(orientation);
    tf.translation() = start;

    Eigen::Vector3d direction = (stop - start).normalized();

    double step = (stop - start).norm() / num_points;
    for (int i{ 0 }; i < num_points; ++i)
    {
        tf.translation() += step * direction;
        task.push_back({ tf, bounds });
    }
    return task;
}

inline double deg2rad(double deg)
{
    return deg * M_PI / 180.0;
}
