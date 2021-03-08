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

std::vector<double> calcMaxJointStep(const std::vector<ocpl::TSR>& task,
                                     const std::vector<ocpl::Bounds>& joint_velocity_limits, const double ee_velocity)
{
    double total_distance{ 0.0 };
    for (std::size_t i{ 1 }; i < task.size(); ++i)
    {
        total_distance += (task[i].tf_nominal.translation() - task[i - 1].tf_nominal.translation()).norm();
    }
    const double dt = (total_distance / ee_velocity) / task.size();

    // calculate max step size from the upper velocity limit and time difference between two waypoints
    std::vector<double> dq_max(joint_velocity_limits.size());
    std::transform(joint_velocity_limits.begin(), joint_velocity_limits.end(), dq_max.begin(),
                   [dt](const ocpl::Bounds& b) { return 1.0 * b.upper * dt; });

    return dq_max;
}
