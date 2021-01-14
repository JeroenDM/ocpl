#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <stdexcept>

#include <Eigen/Geometry>

#include <ocpl_planning/settings.h>
#include <ocpl_tsr/task_space_regions.h>

namespace ocpl
{
/** \brief Helper function to format vectors to pretty print settings. **/
std::string format(const std::vector<int>& v);

/** \brief Helper function to convert SamplerType to string for pretty printing below. **/
std::string format(const SamplerType& t);

/** \brief Pretty printing of planner settings. **/
std::ostream& operator<<(std::ostream& os, const PlannerSettings& ps);

/*************************************************************
 *  Reading a task from an irl file(Industrial Robot Format)
 *************************************************************/

Eigen::Isometry3d parsePose(const std::string& str)
{
    std::stringstream stream(str);
    std::string value;
    std::vector<double> p;  // x, y, z, rx, ry, rz
    while (std::getline(stream, value, ' '))
    {
        if (!value.empty())
            p.push_back(std::stod(value));
    }
    if (p.size() != 6)
        throw std::invalid_argument("ocpl_planning parsing error: the pose string contains more than 6 p.");

    for (auto value : p)
        std::cout << value << ", ";
    std::cout << std::endl;

    using Translation = Eigen::Translation3d;
    using AngleAxis = Eigen::AngleAxisd;
    using Vector = Eigen::Vector3d;

    // clang-format off
    Eigen::Isometry3d t;
    t = Translation(p[0], p[1], p[2]) * AngleAxis(p[3], Vector::UnitX()) * AngleAxis(p[4], Vector::UnitY()) *
        AngleAxis(p[5], Vector::UnitZ());
    // clang-format on

    return t;
}

TSRBounds parseConstraint(const std::string& str)
{
    std::stringstream stream(str);
    std::string value;
    std::vector<double> bounds;  // x_min, y_min, z_min, x_max, y_max, z_max
    std::string con_type;
    while (std::getline(stream, value, ' '))
    {
        if (value.empty())
            continue;
        else if (value == "rpy" || value == "xyz")
            con_type = value;
        else
            bounds.push_back(std::stod(value));
    }
    return TSRBounds{ { bounds[0], bounds[3] }, { bounds[1], bounds[4] }, { bounds[2], bounds[5] } };
}

}  // namespace ocpl
