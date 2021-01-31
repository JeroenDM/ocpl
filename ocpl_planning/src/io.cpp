#include "ocpl_planning/io.h"

#include <iostream>
#include <string>
#include <vector>

#include <ocpl_planning/types.h>
#include <ocpl_planning/numpy_io.h>

namespace ocpl
{
/** \brief Helper function to format vectors to pretty print settings. **/
std::string format(const std::vector<int>& v)
{
    std::string s{};
    for (auto value : v)
    {
        s.append(std::to_string(value));
        s.append(" ");
    }
    return s;
}

/** \brief Helper function to convert SamplerType to string for pretty printing below. **/
std::string format(const SamplerType& t)
{
    switch (t)
    {
        case SamplerType::GRID:
            return "grid";
            break;
        case SamplerType::HALTON:
            return "halton";
            break;
        case SamplerType::RANDOM:
            return "random";
            break;
        default:
            return "";
            break;
    }
}

/** \brief Pretty printing of planner settings. **/
std::ostream& operator<<(std::ostream& os, const PlannerSettings& ps)
{
    // clang-format off
    os << "name: "                  << ps.name << "\n";
    os << "sampler_type: "          << format(ps.sampler_type) << "\n";
    os << "tsr_resolution: "        << format(ps.tsr_resolution) << "\n";
    os << "redundant_joints_resolution: " << format(ps.redundant_joints_resolution) << "\n";
    os << "is_redundant: "          << ps.is_redundant << "\n";
    os << "t_space_batch_size: "    << ps.t_space_batch_size << "\n";
    os << "c_space_batch_size: "    << ps.c_space_batch_size << "\n";
    os << "min_valid_samples: "     << ps.min_valid_samples << "\n";
    os << "max_iters: "             << ps.max_iters << "\n";
    //clang-format on

    return os;
}

void savePath(const std::string& filename, const std::vector<JointPositions>& path)
{
    const size_t num_dof {path.front().size()};
    const size_t num_pts (path.size());

    std::vector<double> data;
    data.reserve(num_pts * num_dof);
    for (auto& q : path)
    {
        data.insert(data.end(), q.begin(), q.end());
    }

    const long unsigned shape[] = { num_pts, num_dof };
    npy::SaveArrayAsNumpy(filename, false, 2, shape, data);
}

std::vector<JointPositions> loadPath(const std::string& filename)
{
    std::vector<unsigned long> shape;
    bool fortran_order;
    std::vector<double> data;

    npy::LoadArrayFromNumpy(filename, shape, fortran_order, data);

    assert(shape.size() == 2); // it should be a 2D array
    assert(fortran_order == false); // fortran order is set to false when saving a path

    const size_t num_dof = shape.at(1);
    const size_t num_pts =shape.at(0);

    std::vector<JointPositions> path;
    path.reserve(num_pts);
    for (size_t waypoint{ 0 }; waypoint < num_pts; ++waypoint)
    {
        size_t offset = waypoint * num_dof;
        path.push_back(std::vector<double>(data.begin() + offset, data.begin() + offset + num_dof));
        
        assert(path.back().size() == num_dof);
    }

    return path;
}

}  // namespace ocpl
