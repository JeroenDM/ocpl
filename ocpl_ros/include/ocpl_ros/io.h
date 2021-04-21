#pragma once

#include <regex>
#include <string>
#include <unordered_map>
#include <vector>

#include <ros/package.h>
#include <eigen_stl_containers/eigen_stl_vector_container.h>

#include <ocpl_planning/planner_base.h>

namespace ocpl
{
typedef std::unordered_map<std::string, std::string> SettingsMap;

namespace impl
{
const std::string THIS_PACKAGE_NAME{ "ocpl_ros" };
const std::string REL_PATH_DATA{ "/data/" };
const std::regex LINE_REGEX{ "^[^:]+: [^:]+$" };  // match key-value pairs seperated by ": "
const std::string LINE_SPLIT_STR{ ": " };

std::pair<std::string, std::string> proccessLine(const std::string& line);
}  // namespace impl

SettingsMap readSettingsFile(const std::string& filename, const std::string& package_name = impl::THIS_PACKAGE_NAME);

std::vector<std::string> readLinesFromFile(const std::string& filename);

template <typename Scalar>
std::vector<Scalar> stringToVector(const std::string& s);

std::string findOrDefault(const SettingsMap& map, const std::string& key, const std::string& default_value)
{
    auto entry = map.find(key);
    if (entry != map.end())
        return entry->second;
    else
        return default_value;
}

size_t findOrDefault(const SettingsMap& map, const std::string& key, const size_t default_value)
{
    auto entry = map.find(key);
    if (entry != map.end())
        return std::stoi(entry->second);
    else
        return default_value;
}

int findOrDefault(const SettingsMap& map, const std::string& key, const int default_value)
{
    auto entry = map.find(key);
    if (entry != map.end())
        return std::stoi(entry->second);
    else
        return default_value;
}

double findOrDefault(const SettingsMap& map, const std::string& key, const double default_value)
{
    auto entry = map.find(key);
    if (entry != map.end())
        return std::stod(entry->second);
    else
        return default_value;
}

bool findOrDefault(const SettingsMap& map, const std::string& key, const bool default_value)
{
    auto entry = map.find(key);
    if (entry != map.end())
    {
        if (entry->second == "true")
        {
            return true;
        }
        else if (entry->second == "false")
        {
            return false;
        }
        else
        {
            throw std::invalid_argument("Cannot convert " + entry->second +
                                        " into bool value.\n Use true or false instead.");
        }
    }
    return default_value;
}

std::ostream& operator<<(std::ostream& os, const JointPositions& q)
{
    for (auto qi : q)
        os << qi << ", ";
    return os;
}

PlannerSettings loadSettingsFromFile(const std::string filename, const std::string& package_name = impl::THIS_PACKAGE_NAME);

/** Simple struct to put waypoint data without parsing converting it to specific objects used by planing algorithms.**/
struct TaskData
{
    EigenSTL::vector_Isometry3d waypoints;
    std::vector<std::array<Bounds, 6>> waypoint_bounds;
};

/** Read and under-constrained end-effector path from a list of waypoints in a csv file.
 *
 * A line in a file constains the a bunch of numbers, represented as named vectors below
 *     p, n, t, lower, upper
 * p: 3 numbers, position
 * n: 3 numbers, z-axis, normal along which an axis pointing out of the robot tool should lie
 * t: 3 numbers, x_axis, tangent vector along the path
 * lower: 6 numbers, lower tolerance bound for position and orientation
 * upper: 6 numbers, upper tolerance bound for position and orientation
 *
 *
 * In total, every line contains 21 numbers separated by ','.
 *
 * The lower and upper bound for orientation tolerance can be for any 3-parameter representation
 * and is expressed relative to the nominal pose of the waypoint.
 *
 * */
std::vector<std::vector<TSR>> readPathsFromCsvFile(const std::string& filename);

}  // namespace ocpl
