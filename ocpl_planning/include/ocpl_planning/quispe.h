#pragma once

#include <queue>

#include <ocpl_planning/planner_base.h>

namespace ocpl
{
namespace quispe
{
using PriorityQueue = std::priority_queue<JointPositions, std::vector<JointPositions>,
                                          std::function<bool(const JointPositions&, const JointPositions&)>>;

class QuispePlanner : public Planner
{
  public:
    std::vector<JointPositions> greedy2(const std::vector<TSR>& task);

    PriorityQueue getLocalSamples(const TSR& tsr, const JointPositions& q_bias, int num_samples);

    JointPositions findChildren(const TSR& tsr, const JointPositions& q_bias, int num_samples = 1);
};
}  // namespace quispe
}  // namespace ocpl
