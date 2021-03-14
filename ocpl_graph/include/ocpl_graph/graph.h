#pragma once

#include <vector>
#include <limits>
#include <ostream>
#include <memory>
#include <functional>
#include <cassert>
#include <cmath>

#include <ocpl_graph/containers.h>

namespace ocpl_graph
{
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

typedef std::function<std::vector<NodePtr>(const NodePtr& node)> SampleFun;

/** \brief A generic graph implementation. The structure is determined by the sample function that is passed in to get
 * the neighbors of a node.
 * TODO Implement iterative grid refining.
 * **/
class Graph
{
    SampleFun sample_fun_;
    size_t num_waypoints_;
    std::vector<std::vector<NodePtr>> nodes_;

  public:
    Graph(SampleFun sample_fun, size_t num_waypoints, std::vector<NodePtr> start_nodes)
      : sample_fun_(sample_fun), num_waypoints_(num_waypoints)
    {
        nodes_.resize(num_waypoints);
        nodes_.at(0) = start_nodes;
    }
    ~Graph() = default;

    const std::vector<NodePtr>& getNeighbors(const NodePtr& node);
    const std::vector<NodePtr>& getStartNodes();
    const bool isGoal(const NodePtr& node) const;
    size_t size() const
    {
        return num_waypoints_;
    }

    std::vector<NodePtr> extract_solution(const NodePtr& goal) const;
    std::vector<NodePtr> extract_partial_solution() const;
};

/** \brief Find shortest path in a directed acyclic graph. **/
std::vector<NodePtr> shortest_path_dag(Graph& graph, std::function<double(const NodePtr, const NodePtr)> cost_function,
                                       BaseContainer<NodePtr>& cont, int max_runtime_seconds = 5);

std::ostream& operator<<(std::ostream& os, const Node& node);

}  // namespace ocpl_graph
