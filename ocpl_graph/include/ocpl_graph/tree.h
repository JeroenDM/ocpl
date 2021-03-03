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

class Graph
{
  protected:
    bool sample_locally_{ true };

  public:
    virtual const std::vector<NodePtr>& getNeighbors(const NodePtr& node) = 0;
    virtual const std::vector<NodePtr>& getStartNodes() = 0;
    virtual const bool isGoal(const NodePtr& node) const = 0;
    virtual size_t size() const = 0;

    virtual std::vector<NodePtr> extract_solution(const NodePtr& goal) const = 0;
    virtual std::vector<NodePtr> extract_partial_solution() const = 0;
    virtual void setSampleLocally(bool value)
    {
        sample_locally_ = value;
    };
};

class DAGraph : public Graph
{
    std::vector<std::vector<NodePtr>> nodes_;
    size_t num_waypoints_;

  public:
    DAGraph(std::vector<std::vector<NodePtr>>& nodes) : nodes_(nodes), num_waypoints_(nodes.size())
    {
    }
    ~DAGraph() = default;

    virtual const std::vector<NodePtr>& getNeighbors(const NodePtr& node) override;
    virtual const std::vector<NodePtr>& getStartNodes() override;
    virtual const bool isGoal(const NodePtr& node) const override;
    virtual size_t size() const override
    {
        return num_waypoints_;
    }

    virtual std::vector<NodePtr> extract_solution(const NodePtr& goal) const override;
    virtual std::vector<NodePtr> extract_partial_solution() const override;
};

typedef std::function<std::vector<NodePtr>(const NodePtr& node, size_t num_samples)> SampleFun;

class Tree : public Graph
{
    SampleFun sample_fun_;
    size_t num_waypoints_;
    std::vector<std::vector<NodePtr>> nodes_;

  public:
    Tree(SampleFun sample_fun, size_t num_waypoints, std::vector<NodePtr> start_nodes)
      : sample_fun_(sample_fun), num_waypoints_(num_waypoints)
    {
        nodes_.resize(num_waypoints);
        nodes_.at(0) = start_nodes;
    }
    ~Tree() = default;

    virtual const std::vector<NodePtr>& getNeighbors(const NodePtr& node) override;
    virtual const std::vector<NodePtr>& getStartNodes() override;
    virtual const bool isGoal(const NodePtr& node) const override;
    virtual size_t size() const override
    {
        return num_waypoints_;
    }

    virtual std::vector<NodePtr> extract_solution(const NodePtr& goal) const override;
    virtual std::vector<NodePtr> extract_partial_solution() const override;
};

/** \brief Find shortest path in a directed acyclic graph. **/
std::vector<NodePtr> shortest_path_dag(Graph& graph, std::function<double(const NodePtr, const NodePtr)> cost_function);

std::ostream& operator<<(std::ostream& os, const Node& node);

}  // namespace ocpl
