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

SettingsMap readSettingsFile(const std::string& filename, const std::string& package_name)
{
    // Find the data folder in the current package where we expect the file to be located
    std::string path = ros::package::getPath(package_name);
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

std::vector<std::string> readLinesFromFile(const std::string& filename, const std::string& package_name)
{
    // Find the data folder in the current package where we expect the file to be located
    std::string path = ros::package::getPath(package_name);
    path.append(impl::REL_PATH_DATA);
    path.append(filename);

    std::ifstream file(path);

    if (!file.is_open())
    {
        throw std::runtime_error({ "File [" + path + "] not found." });
    }

    std::string line;
    std::vector<std::string> result;
    while (std::getline(file, line))
    {
        result.emplace_back(line);
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

PlannerSettings loadSettingsFromFile(const std::string filename, const std::string& package_name)
{
    auto map = readSettingsFile(filename, package_name);

    PlannerSettings s;
    s.name = findOrDefault(map, "name", std::string{ "default_name" });
    s.is_redundant = findOrDefault(map, "is_redundant", s.is_redundant);
    s.t_space_batch_size = findOrDefault(map, "t_space_batch_size", s.t_space_batch_size);
    s.c_space_batch_size = findOrDefault(map, "c_space_batch_size", s.c_space_batch_size);
    s.min_shots = findOrDefault(map, "min_shots", s.min_shots);
    s.max_iters = findOrDefault(map, "max_iters", s.max_iters);
    s.min_valid_samples = findOrDefault(map, "min_valid_samples", s.min_valid_samples);
    s.cspace_delta = findOrDefault(map, "cspace_delta", s.cspace_delta);
    s.state_cost_weight = findOrDefault(map, "state_cost_weight", s.state_cost_weight);
    s.debug = findOrDefault(map, "debug", s.debug);
    s.timeout = findOrDefault(map, "timeout", s.timeout);

    // mapping from strings in settings file to planner types
    static std::map<std::string, PlannerType> type_mapping{
        { "local_dfs", PlannerType::LOCAL_DFS }, { "local_best_first_dfs", PlannerType::LOCAL_BEST_FIRST_DFS },
        { "global", PlannerType::GLOBAL },       { "global_dfs", PlannerType::GLOBAL_DFS },
        { "greedy", PlannerType::GREEDY },       { "bigreedy", PlannerType::BIGREEDY },
        { "rrtlike", PlannerType::RRTLIKE },
    };

    auto entry = map.find("planner_type");
    if (entry != map.end())
    {
        auto type_pair = type_mapping.find(entry->second);
        if (type_pair != type_mapping.end())
        {
            s.type = type_pair->second;
        }
        else
        {
            throw std::invalid_argument("Invalid planner type: " + entry->second);
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

std::vector<std::vector<TSR>> readPathsFromCsvFile(const std::string& filename)
{
    std::ifstream file(ros::package::getPath("ocpl_ros") + "/data/" + filename);

    if (!file.is_open())
    {
        std::runtime_error("Failed to read file " + filename);
    }

    std::vector<TaskData> tasks;
    EigenSTL::vector_Isometry3d waypoints;
    std::vector<std::array<Bounds, 6>> wp_bounds;
    std::string line;
    while (std::getline(file, line))
    {
        if (line == "")
        {
            if (!waypoints.empty())
            {
                tasks.push_back({ waypoints, wp_bounds });
                waypoints.clear();
                wp_bounds.clear();
            }
        }
        else
        {
            std::stringstream stream(line);
            std::string number;
            std::array<double, 21> v;
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

            std::array<Bounds, 6> bounds;
            for (std::size_t i = 0; i < 6; ++i)
            {
                bounds.at(i) = { v[9 + i], v[9 + 6 + i] };
            }

            waypoints.push_back(pose);
            wp_bounds.push_back(bounds);
        }
    }
    if (!waypoints.empty())
    {
        tasks.push_back({ waypoints, wp_bounds });
    }

    std::vector<std::vector<TSR>> paths;
    for (auto task_data : tasks)
    {
        std::vector<TSR> current_task;
        for (auto waypoint : task_data.waypoints)
        {
            current_task.push_back(TSR{ waypoint, TSRBounds{} });
        }
        for (std::size_t wp{ 0 }; wp < current_task.size(); ++wp)
        {
            std::vector<Bounds> temp(task_data.waypoint_bounds.at(wp).begin(), task_data.waypoint_bounds.at(wp).end());
            current_task[wp].bounds.fromVector(temp);
        }
        paths.push_back(current_task);
    }

    return paths;
}

}  // namespace ocpl
