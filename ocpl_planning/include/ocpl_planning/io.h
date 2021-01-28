#pragma once

#include <iostream>
#include <string>
#include <vector>

#include <ocpl_planning/types.h>

namespace ocpl
{
/** \brief Helper function to format vectors to pretty print settings. **/
std::string format(const std::vector<int>& v);

/** \brief Helper function to convert SamplerType to string for pretty printing below. **/
std::string format(const SamplerType& t);

/** \brief Pretty printing of planner settings. **/
std::ostream& operator<<(std::ostream& os, const PlannerSettings& ps);
}  // namespace ocpl
