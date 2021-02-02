#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <stdexcept>
#include <map>
#include <functional>

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

template <typename T>
void debug(const T& msg)
{
    std::cout << msg << "\n";
}

std::pair<std::string, std::string> splitOnFirstSpace(const std::string& str)
{
    size_t first_space_index = str.find(' ');
    std::string first = str.substr(0, first_space_index);
    if (first_space_index < str.size())
    {
        std::string second = str.substr(first_space_index, str.size() - first_space_index);
        return { first, second };
    }
    else
    {
        return { first, {} };
    }
}

/** \brief Parse a space separated vector of floating point values
 *
 * A vector is expected to be a list of numbers separated by spaces.
 * The result is an std::vector with doubles.
 *
 * **/
std::vector<double> parseVector(const std::string& str)
{
    std::stringstream stream(str);
    std::string value;
    std::vector<double> v;
    while (std::getline(stream, value, ' '))
    {
        if (!value.empty())
            v.push_back(std::stod(value));
    }
    return v;
}

Eigen::Isometry3d parsePose(const std::string& str)
{
    auto p = parseVector(str);
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



enum class Section
{
    VARS = 0,
    COMMANDS = 1,
    CONS = 2
};

const std::map<std::string, Section> SECTIONS{ { "variables", Section::VARS },
                                               { "commands", Section::COMMANDS },
                                               { "constraints", Section::CONS } };

struct Variable
{
};
struct Pose : public Variable
{
    Eigen::Isometry3d tf;
};
struct Config : public Variable
{
    std::vector<double> positions;
};

typedef std::shared_ptr<Variable> VariablePtr;
typedef std::shared_ptr<Pose> PosePtr;
typedef std::shared_ptr<Config> ConfigPtr;

struct ParserState
{
    Section section;
    Eigen::Isometry3d current_reference_frame;
    std::map<std::string, VariablePtr> env{};
};

typedef std::function<ParserState(const std::string&, const ParserState&)> ActionFun;

// ParserState config(const std::string& args, const ParserState& state)
// {
//     auto p = splitOnFirstSpace(args);
//     auto name = p.first;
//     auto values = p.second;
//     auto q = parseVector(values);
//     ParserState new_state = state;
//     new_state.env[name] = std::make_shared<Config>(q);
//     return new_state;
// }

// ParserState pose(const std::string& args, const ParserState& state)
// {
//     auto p = splitOnFirstSpace(args);
//     auto name = p.first;
//     auto values = p.second;
//     auto tf = parsePose(values);
//     ParserState new_state = state;
//     new_state.env[name] = std::make_shared<Pose>(tf);
//     return new_state;
// }

// const std::map<std::string, ActionFun> actions{ { "config", config } };

ParserState parseLine(const std::string& line, const ParserState& state)
{
    if (line.empty())
    {
        return state;
    }
    // size_t first_space_index = line.find(' ');
    // // get the current action, this is the string before the first space
    // std::string action = line.substr(0, first_space_index);
    auto split = splitOnFirstSpace(line);
    auto action = split.first;
    auto properties = split.second;

    debug("Current action: " + action);

    ParserState new_state = state;
    // are we entering a new section?
    auto it = SECTIONS.find(line);
    if (it != SECTIONS.end())
    {
        new_state.section = it->second;
        debug("\tnew state: " + it->first);
        return new_state;
    }

    // std::string action_properties = line.substr(first_space_index, line.size() - first_space_index);
    debug("\taction properties: " + properties);

    return state;
}

void parseIrlTask(const std::string& content)
{
    ParserState state{ {}, Eigen::Isometry3d::Identity() };

    std::stringstream stream(content);
    std::string line;
    while (std::getline(stream, line))
    {
        // std::cout << line << "\n";
        state = parseLine(line, state);
    }
}

}  // namespace ocpl
