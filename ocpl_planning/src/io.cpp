#include "ocpl_planning/io.h"

#include <iostream>
#include <string>
#include <vector>

#include <ocpl_planning/types.h>

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

}  // namespace ocpl
