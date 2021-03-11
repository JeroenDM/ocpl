#pragma once

#include <regex>
#include <string>
#include <unordered_map>
#include <vector>

#include <ros/package.h>

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

SettingsMap readSettingsFile(const std::string& filename);

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

PlannerSettings loadSettingsFromFile(const std::string filename);

}  // namespace ocpl
