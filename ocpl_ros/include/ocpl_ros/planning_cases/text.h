#pragma once

#include <algorithm>
#include <fstream>
#include <sstream>
#include <vector>

#include <Eigen/Core>

#include <ros/package.h>
#include <eigen_stl_containers/eigen_stl_containers.h>

#include <ocpl_ros/planning_cases/util.h>

#include <exception>

namespace text
{
namespace impl
{
/** \brief Read the data from the puzzle piece from the Descartes tutorials.
 * code copied from:
 * https://github.com/ros-industrial-consortium/descartes_tutorials/blob/master/descartes_tutorials/src/tutorial2.cpp
 *
 * (And the corresponding csv file is also copied from these tutorials.)
 * **/
static EigenSTL::vector_Isometry3d readPathFromFile(const std::string& filename)
{
    std::ifstream file(filename);

    if (!file.is_open())
    {
        std::runtime_error("Failed to read file " + filename);
    }

    EigenSTL::vector_Isometry3d path;
    std::string line;
    while (std::getline(file, line))
    {
        std::stringstream stream(line);
        std::string number;
        std::array<double, 9> v;
        std::size_t i{ 0 };
        while (std::getline(stream, number, ','))
        {
            v.at(i) = std::stod(number);
            i++;
        }
        Eigen::Vector3d pos, x_axis, y_axis, z_axis;
        pos << v[0], v[1], v[2];
        z_axis << v[3], v[4], v[5];  // normal vector
        x_axis << v[6], v[7], v[8];  // tangent vector along the path

        x_axis = x_axis.normalized();
        y_axis = z_axis.cross(x_axis).normalized();
        z_axis = z_axis.normalized();

        Eigen::Isometry3d pose;
        pose.matrix().col(0).head<3>() = x_axis;
        pose.matrix().col(1).head<3>() = y_axis;
        pose.matrix().col(2).head<3>() = z_axis;
        pose.matrix().col(3).head<3>() = pos;

        path.push_back(pose);
    }

    return path;
}
}  // namespace impl

std::vector<ocpl::TSR> waypoints(const Eigen::Isometry3d& offset)
{
    ocpl::TSRBounds bounds{ { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { -M_PI, M_PI } };
    auto task_tfs = impl::readPathFromFile(ros::package::getPath("ocpl_ros") + "/data/text.csv");

    Eigen::AngleAxisd rotation(-M_PI_2, Eigen::Vector3d::UnitZ());
    Eigen::Translation3d translation(0.5, 2.0, 0.15);

    std::vector<ocpl::TSR> task;
    for (auto& pose : task_tfs)
    {
        task.push_back({ offset * (translation * rotation * pose), bounds });
    }
    return task;
}

}  // namespace text
