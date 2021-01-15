#include "ocpl_planning/prm.h"

#include <cmath>
#include <cassert>
#include <functional>
#include <vector>
#include <iostream>

#include <ocpl_graph/tree.h>

namespace ocpl
{
std::vector<JointPositions> interpolate(const JointPositions& from, const JointPositions& to, double max_c_step)
{
    assert(from.size() == to.size());

    const std::size_t DOF{ from.size() };

    // calculate the number of interpolation points
    double j_step{ 0.0 };
    for (std::size_t i{ 0 }; i < DOF; ++i)
    {
        j_step = std::max(j_step, std::abs(to[i] - from[i]));
    }
    int num_steps = (int)std::ceil(j_step / max_c_step);

    // construct the interpolated path
    double ds = 1.0 / (num_steps + 1);
    std::vector<JointPositions> path;
    for (double s{ 0.0 }; s <= 1; s += ds)
    {
        JointPositions q(DOF);
        for (std::size_t i{ 0 }; i < DOF; ++i)
            q[i] = from[i] + s * (to[i] - from[i]);
        path.push_back(q);
    }
    return path;
}

void tryToConnectNodes(Tree& tree, const std::vector<NodePtr> nodes,
                       const ompl::NearestNeighborsLinear<NodePtr>& nn_graph,
                       std::function<bool(const JointPositions&, const JointPositions&)> checkLocalPath,
                       std::function<double(const JointPositions&, const JointPositions&)> costFun)
{
    static constexpr size_t TRY_CONNECT{ 50 };
    for (NodePtr current : nodes)
    {
        std::vector<NodePtr> nb;
        nb.reserve(TRY_CONNECT);
        nn_graph.nearestK(current, TRY_CONNECT, nb);
        for (auto neighbour : nb)
        {
            if (checkLocalPath(current->data, neighbour->data))
            {
                double cost = costFun(current->data, neighbour->data);
                tree[current].push_back(Edge{ neighbour, cost });
                tree[neighbour].push_back(Edge{ current, cost });
            }
        }
        std::cout << "Found " << tree[current].size() << " connections.\n";
    }
}

Roadmap createRoadmap(StateSamplerFun roadSampler, StateSamplerFun startSampler, StateSamplerFun goalSampler,
                      const RoadmapProperties& rp)
{
    std::vector<NodePtr> start_nodes;
    std::vector<NodePtr> goal_nodes;
    std::vector<NodePtr> nodes;
    start_nodes.reserve(rp.num_start_samples);
    goal_nodes.reserve(rp.num_goal_samples);
    nodes.reserve(rp.num_road_samples);

    for (JointPositions q : startSampler(rp.num_start_samples))
    {
        goal_nodes.push_back(std::make_shared<Node>(q, 0.0));
    }

    for (JointPositions q : goalSampler(rp.num_goal_samples))
    {
        goal_nodes.push_back(std::make_shared<Node>(q, 0.0));
    }

    for (JointPositions q : roadSampler(rp.num_road_samples))
    {
        nodes.push_back(std::make_shared<Node>(q, 0.0));
    }

    Roadmap roadmap;
    roadmap.nearest_neighbours.setDistanceFunction(CSpaceDistance);
    roadmap.nearest_neighbours.add(nodes);
    roadmap.start_nodes = start_nodes;
    roadmap.goal_nodes = goal_nodes;
    roadmap.nodes = nodes;

    return roadmap;
}

Tree connectRoadmap(const Roadmap& roadmap,
                    std::function<bool(const JointPositions&, const JointPositions&)> checkLocalPath,
                    std::function<double(const JointPositions&, const JointPositions&)> costFun)
{
    Tree tree;
    tryToConnectNodes(tree, roadmap.nodes, roadmap.nearest_neighbours, checkLocalPath, costFun);
    tryToConnectNodes(tree, roadmap.start_nodes, roadmap.nearest_neighbours, checkLocalPath, costFun);
    tryToConnectNodes(tree, roadmap.goal_nodes, roadmap.nearest_neighbours, checkLocalPath, costFun);
    return tree;
}

}  // namespace ocpl
