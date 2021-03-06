#pragma once

#include <iostream>
#include <string>
#include <vector>

#include <ocpl_planning/types.h>
#include <ocpl_planning/planner_base.h>  // PlannerSettings

namespace ocpl
{
/** \brief Helper function to format vectors to pretty print settings. **/
std::string format(const std::vector<int>& v);

/** \brief Helper function to convert SamplerType to string for pretty printing below. **/
std::string format(const SamplerType& t);

/** \brief Pretty printing of planner settings. **/
std::ostream& operator<<(std::ostream& os, const PlannerSettings& ps);

template <typename T>
std::ostream& operator<<(std::ostream& os, const std::vector<T> v)
{
    os << "( ";
    for (auto el : v)
        os << el << ", ";
    os << " )";
    return os;
}

/** \brief Write the path to a simple csv file. **/
void savePath(const std::string& filename, const std::vector<JointPositions>& path);

/** \brief Read a path that was saved as a numpy 2D array (num_pts x num_dof). **/
std::vector<JointPositions> loadPath(const std::string& filename);

}  // namespace ocpl
