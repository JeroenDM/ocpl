#pragma once

#include <exception>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <regex>
#include <unordered_map>

#include <ros/package.h>

namespace ocpl
{
typedef std::unordered_map<std::string, std::string> SettingsMap;

namespace impl
{
const std::string THIS_PACKAGE_NAME{ "ocpl_ros" };
const std::string REL_PATH_DATA{ "/data/" };
const std::regex LINE_REGEX{ "^[^:]+: [^:]+$" }; // match key-value pairs seperated by ": "

std::pair<std::string, std::string> proccessLine(const std::string& line)
{
    // black magic as input validation
    if (!std::regex_match(line.begin(), line.end(), LINE_REGEX))
    {
        throw std::runtime_error({ "Line in settings file has syntax error: " + line });
    }
    std::stringstream ss(line);
    std::string key, value;
    ss >> key >> value;
    key.pop_back();  // remove the trailing ":"
    return { key, value };
}
}  // namespace impl

SettingsMap readSettingsFile(const std::string& filename)
{
    // Find the data folder in the current package where we expect the file to be located
    std::string path = ros::package::getPath(impl::THIS_PACKAGE_NAME);
    path.append(impl::REL_PATH_DATA);
    path.append(filename);

    std::ifstream file(path);

    if (!file.is_open())
    {
        throw std::runtime_error({ "Planner settings file [" + path + "] not found." });
    }

    std::string line;
    SettingsMap result;
    while (std::getline(file, line))
    {
        std::cout << line << "\n";
        auto key_value = impl::proccessLine(line);
        result[key_value.first] = key_value.second;
    }

    return result;
}

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

double findOrDefault(const SettingsMap& map, const std::string& key, const double default_value)
{
    auto entry = map.find(key);
    if (entry != map.end())
        return std::stod(entry->second);
    else
        return default_value;
}

std::ostream& operator<<(std::ostream& os, const JointPositions& q)
{
    for (auto qi : q)
        os << qi << ", ";
    return os;
}

}  // namespace ocpl
