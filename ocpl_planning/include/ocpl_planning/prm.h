#include <functional>
#include <vector>

#include <ocpl_graph/tree.h>

#include <ompl/datastructures/NearestNeighborsLinear.h>

namespace ocpl
{
typedef std::vector<double> JointPositions;
typedef std::function<std::vector<JointPositions>(int)> StateSamplerFun;
// namespace defaults
// {
// static constexpr double MAX_C_STEP{ 0.1 };  // maximum motion for a single joint when interpolating
// }

inline double CSpaceDistance(NodePtr n1, NodePtr n2)
{
    double cost{ 0.0 };
    for (int i = 0; i < n1->data.size(); ++i)
    {
        cost += std::abs(n1->data[i] - n2->data[i]);
    }
    // if (cost > 1.0)
    //     cost = std::numeric_limits<double>::max();
    return cost;
}

std::vector<JointPositions> interpolate(const JointPositions& from, const JointPositions& to, double max_c_step = 0.1);

void tryToConnectNodes(Tree& tree, const std::vector<NodePtr> nodes,
                       const ompl::NearestNeighborsLinear<NodePtr>& nn_graph,
                       std::function<bool(const JointPositions&, const JointPositions&)> checkLocalPath,
                       std::function<double(const JointPositions&, const JointPositions&)> costFun);

struct Roadmap
{
    ompl::NearestNeighborsLinear<NodePtr> nearest_neighbours;
    std::vector<NodePtr> start_nodes;
    std::vector<NodePtr> goal_nodes;
    std::vector<NodePtr> nodes;
};

struct RoadmapProperties
{
    size_t num_road_samples, num_start_samples, num_goal_samples;
};

Roadmap createRoadmap(StateSamplerFun roadSampler, StateSamplerFun startSampler, StateSamplerFun goalSampler,
                      const RoadmapProperties& rp);

Tree connectRoadmap(const Roadmap& roadmap,
                    std::function<bool(const JointPositions&, const JointPositions&)> checkLocalPath,
                    std::function<double(const JointPositions&, const JointPositions&)> costFun);

}  // namespace ocpl
