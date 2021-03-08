#include <ocpl_ros/io.h>

#include <exception>
#include <fstream>
#include <iostream>
#include <sstream>

namespace ocpl
{
namespace impl
{
std::pair<std::string, std::string> proccessLine(const std::string& line)
{
    // black magic as input validation
    if (!std::regex_match(line.begin(), line.end(), LINE_REGEX))
    {
        throw std::runtime_error({ "Line in settings file has syntax error: " + line });
    }
    auto split_index = line.find(LINE_SPLIT_STR);
    std::string key = line.substr(0, split_index);
    std::string value = line.substr(split_index + LINE_SPLIT_STR.size(), line.size());
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

template <typename Scalar>
std::vector<Scalar> stringToVector(const std::string& s)
{
    std::stringstream stream(s);
    Scalar number;
    std::vector<Scalar> v;
    while (stream >> number)
    {
        v.push_back(number);
    }
    return v;
}

template std::vector<int> stringToVector(const std::string& s);

PlannerSettings loadSettingsFromFile(const std::string filename)
{
    auto map = readSettingsFile(filename);

    PlannerSettings s;
    s.is_redundant = findOrDefault(map, "is_redundant", s.is_redundant);
    s.t_space_batch_size = findOrDefault(map, "t_space_batch_size", s.t_space_batch_size);
    s.c_space_batch_size = findOrDefault(map, "c_space_batch_size", s.c_space_batch_size);
    s.max_iters = findOrDefault(map, "max_iters", s.max_iters);
    s.min_valid_samples = findOrDefault(map, "min_valid_samples", s.min_valid_samples);
    s.cspace_delta = findOrDefault(map, "cspace_delta", s.cspace_delta);
    s.state_cost_weight = findOrDefault(map, "state_cost_weight", s.state_cost_weight);

    auto entry = map.find("planner_type");
    if (entry != map.end())
    {
        if (entry->second == "local")
        {
            s.type = PlannerType::LOCAL;
        }
        else /* (entry->second == "global") */
        {
            s.type = PlannerType::GLOBAL;
        }
    }

    entry = map.find("sampler_type");
    if (entry != map.end())
    {
        if (entry->second == "halton")
        {
            s.sampler_type = SamplerType::HALTON;
        }
        else if (entry->second == "grid")
        {
            s.sampler_type = SamplerType::GRID;
        }
        else /*(entry->second == "random") */
        {
            s.sampler_type = SamplerType::RANDOM;
        }
    }

    if (s.sampler_type == SamplerType::GRID)
    {
        s.redundant_joints_resolution = stringToVector<int>(map["redundant_joints_resolution"]);
        s.tsr_resolution = stringToVector<int>(map["tsr_resolution"]);

        if (s.is_redundant && s.redundant_joints_resolution.empty())
        {
            throw std::runtime_error("A redundant robot + grid sampling needs a the redundant_joints_resolution "
                                     "setting.");
        }
        if (s.tsr_resolution.size() != 6)
        {
            throw std::runtime_error("Grid sampling needs the tsr_resolution setting with 6 values");
        }
    }

    return s;
}

}  // namespace ocpl
