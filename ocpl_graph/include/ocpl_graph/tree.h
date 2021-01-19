#pragma once

#include <vector>
#include <limits>
#include <ostream>
#include <unordered_map>
#include <memory>
#include <functional>

namespace ocpl
{
struct Node
{
    std::vector<double> data;
    double cost{ 0 };  // state cost of this node

    // attributes below are for the tree search algorithms
    double dist{ std::numeric_limits<double>::max() };
    std::shared_ptr<Node> parent{ nullptr };
    bool visited{ false };
    std::size_t waypoint_index{0};

    explicit Node(std::vector<double> d, double c) : data(std::move(d)), cost(c)
    {
    }

    void reset();
};
typedef std::shared_ptr<Node> NodePtr;

struct Edge
{
    NodePtr child{ nullptr };
    double cost = { 0 };

    Edge(NodePtr child_, double cost_) : child(std::move(child_)), cost(cost_)
    {
    }
};

/* Tree data structure
 *
 * An unordered map is used as a hash table to store a tree as a linked list.
 */
typedef std::unordered_map<NodePtr, std::vector<Edge>> Tree;

/* Search tree for shortest path
 *
 * Implementation of Dijkstra's algorithm using an std::priority_queue.
 */
std::vector<NodePtr> _extract_path(const Tree& tree, NodePtr goal);
std::vector<NodePtr> _extract_path(NodePtr goal);
std::vector<NodePtr> _extract_partial_solution(const std::vector<std::vector<NodePtr>>& nodes);
std::vector<NodePtr> shortest_path(Tree& tree, NodePtr start, NodePtr goal);
std::vector<NodePtr> shortest_path(const Tree& tree, std::vector<NodePtr>& start_nodes, std::vector<NodePtr>& goal_nodes);

/** \brief Find shortest path in a directed acyclic graph. **/
std::vector<NodePtr> shortest_path_dag(const std::vector<std::vector<NodePtr>>& nodes, std::function<double(const NodePtr, const NodePtr)> cost_function);

std::ostream& operator<<(std::ostream& os, const Node& node);

std::ostream& operator<<(std::ostream& os, const Edge& edge);

std::ostream& operator<<(std::ostream& os, const std::vector<Edge>& edges);

}  // namespace ocpl
