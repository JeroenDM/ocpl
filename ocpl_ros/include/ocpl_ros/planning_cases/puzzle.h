#pragma once

#include <algorithm>
#include <fstream>
#include <sstream>
#include <vector>

#include <Eigen/Core>

#include <ros/package.h>
#include <eigen_stl_containers/eigen_stl_containers.h>

#include <ocpl_ros/planning_cases/util.h>

namespace puzzle
{
namespace impl
{
/** \brief Read the data from the puzzle piece from the Descartes tutorials.
 * code copied from:
 * https://github.com/ros-industrial-consortium/descartes_tutorials/blob/master/descartes_tutorials/src/tutorial2.cpp
 *
 * (And the corresponding csv file is also copied from these tutorials.)
 * **/
static EigenSTL::vector_Isometry3d makePuzzleToolPoses()
{
    EigenSTL::vector_Isometry3d path;  // results
    std::ifstream indata;              // input file

    // You could load your parts from anywhere, but we are transporting them with the git repo
    std::string filename = ros::package::getPath("ocpl_ros") + "/data/puzzle_descartes.csv";

    // In a non-trivial app, you'll of course want to check that calls like 'open' succeeded
    indata.open(filename);

    std::string line;
    int lnum = 0;
    while (std::getline(indata, line))
    {
        ++lnum;
        if (lnum < 3)
            continue;

        std::stringstream lineStream(line);
        std::string cell;
        Eigen::Matrix<double, 6, 1> xyzijk;
        int i = -2;
        while (std::getline(lineStream, cell, ','))
        {
            ++i;
            if (i == -1)
                continue;

            xyzijk(i) = std::stod(cell);
        }

        Eigen::Vector3d pos = xyzijk.head<3>();
        pos = pos / 1000.0;  // Most things in ROS use meters as the unit of length. Our part was exported in mm.
        Eigen::Vector3d norm = xyzijk.tail<3>();
        norm.normalize();

        // This code computes two extra directions to turn the normal direction into a full defined frame. Descartes
        // will search around this frame for extra poses, so the exact values do not matter as long they are valid.
        Eigen::Vector3d temp_x = (-1 * pos).normalized();
        Eigen::Vector3d y_axis = (norm.cross(temp_x)).normalized();
        Eigen::Vector3d x_axis = (y_axis.cross(norm)).normalized();
        Eigen::Isometry3d pose;
        pose.matrix().col(0).head<3>() = x_axis;
        pose.matrix().col(1).head<3>() = y_axis;
        pose.matrix().col(2).head<3>() = norm;
        pose.matrix().col(3).head<3>() = pos;

        path.push_back(pose);
    }
    indata.close();

    return path;
}
}  // namespace impl

std::vector<ocpl::Bounds> tsrBounds()
{
    return { { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { -M_PI, M_PI } };
}

std::vector<ocpl::TSR> waypoints(double x_offset = 0.8)
{
    ocpl::TSRBounds bounds{ { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { -M_PI, M_PI } };
    auto task_tfs = impl::makePuzzleToolPoses();

    std::vector<ocpl::TSR> task;
    for (auto& pose : task_tfs)
    {
        pose.translation().x() += x_offset;
        task.push_back({ pose, bounds });
    }
    return task;
}

}  // namespace puzzle
