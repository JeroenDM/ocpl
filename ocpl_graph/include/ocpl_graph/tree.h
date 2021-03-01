#pragma once

#include <vector>
#include <limits>
#include <ostream>
#include <memory>
#include <functional>
#include <cassert>
#include <cmath>

namespace ocpl
{
namespace math
{
inline double norm2Diff(const std::vector<double>& n1, const std::vector<double>& n2)
{
    assert(n1.size() == n2.size());

    double cost{ 0.0 };
    for (int i = 0; i < n1.size(); ++i)
    {
        cost += std::sqrt((n1[i] - n2[i]) * (n1[i] - n2[i]));
    }
    return cost;
}
}  // namespace math

struct Node
{
    std::vector<double> data;
    double cost{ 0 };  // state cost of this node

    // attributes below are for the tree search algorithms
    double dist{ std::numeric_limits<double>::max() };
    std::shared_ptr<Node> parent{ nullptr };
    bool visited{ false };
    std::size_t waypoint_index{ 0 };

    explicit Node(std::vector<double> d, double c) : data(std::move(d)), cost(c)
    {
    }
};
typedef std::shared_ptr<Node> NodePtr;

std::vector<NodePtr> _extract_path(NodePtr goal);
std::vector<NodePtr> _extract_partial_solution(const std::vector<std::vector<NodePtr>>& nodes);

/** \brief Find shortest path in a directed acyclic graph. **/
std::vector<NodePtr> shortest_path_dag(const std::vector<std::vector<NodePtr>>& nodes,
                                       std::function<double(const NodePtr, const NodePtr)> cost_function);

std::ostream& operator<<(std::ostream& os, const Node& node);

}  // namespace ocpl
