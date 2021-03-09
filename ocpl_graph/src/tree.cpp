#include <ocpl_graph/tree.h>

#include <algorithm>
#include <queue>
#include <iostream>
#include <cmath>  // isnan
#include <cassert>
#include <chrono>

#include <ocpl_graph/containers.h>

namespace ocpl
{
struct compareNodesFunction
{
    bool operator()(NodePtr n1, NodePtr n2) const
    {
        return (*n2).dist < (*n1).dist;
    }
};

const std::vector<NodePtr>& DAGraph::getNeighbors(const NodePtr& node)
{
    return nodes_.at(node->waypoint_index + 1);
}
const std::vector<NodePtr>& DAGraph::getStartNodes()
{
    return nodes_.at(0);
}
const bool DAGraph::isGoal(const NodePtr& node) const
{
    return node->waypoint_index == (num_waypoints_ - 1);
}

std::vector<NodePtr> DAGraph::extract_solution(const NodePtr& goal) const
{
    std::vector<NodePtr> path;
    path.push_back(goal);
    NodePtr node = goal;
    while (true)
    {
        if (node->parent == nullptr)
        {
            break;
        }
        else
        {
            node = node->parent;
            path.insert(path.begin(), node);
        }
    }

    return path;
}

std::vector<NodePtr> DAGraph::extract_partial_solution() const
{
    for (std::size_t pi{ nodes_.size() - 1 }; pi >= 0; --pi)
    {
        auto pt_nodes = nodes_[pi];
        auto node_iter = std::min_element(pt_nodes.begin(), pt_nodes.end(),
                                          [](const NodePtr& a, const NodePtr& b) { return a->dist < b->dist; });
        if (node_iter != pt_nodes.end() && (*node_iter)->parent != nullptr)
        {
            std::cout << "Found partial path up until pt index: " << pi << ".\n";
            return extract_solution(*node_iter);
        }
    }
    std::cout << "Also could not extract partial solution.\n";
    return {};
}

/** Tree implementation **/
const std::vector<NodePtr>& Tree::getNeighbors(const NodePtr& node)
{
    if (sample_locally_)
    {
        // override previous samples for the sampled waypoint
        nodes_.at(node->waypoint_index + 1) = sample_fun_(node, 300);
    }
    return nodes_.at(node->waypoint_index + 1);
}
const std::vector<NodePtr>& Tree::getStartNodes()
{
    return nodes_.at(0);
}
const bool Tree::isGoal(const NodePtr& node) const
{
    return node->waypoint_index == (num_waypoints_ - 1);
}

std::vector<NodePtr> Tree::extract_solution(const NodePtr& goal) const
{
    std::vector<NodePtr> path;
    path.push_back(goal);
    NodePtr node = goal;
    while (true)
    {
        if (node->parent == nullptr)
        {
            break;
        }
        else
        {
            node = node->parent;
            path.insert(path.begin(), node);
        }
    }

    return path;
}

std::vector<NodePtr> Tree::extract_partial_solution() const
{
    for (std::size_t pi{ nodes_.size() - 1 }; pi >= 0; --pi)
    {
        auto pt_nodes = nodes_[pi];
        auto node_iter = std::min_element(pt_nodes.begin(), pt_nodes.end(),
                                          [](const NodePtr& a, const NodePtr& b) { return a->dist < b->dist; });
        if (node_iter != pt_nodes.end() && (*node_iter)->parent != nullptr)
        {
            std::cout << "Found partial path up until pt index: " << pi << ".\n";
            return extract_solution(*node_iter);
        }
    }
    std::cout << "Also could not extract partial solution.\n";
    return {};
}

/** Graph traversal algorithm **/

std::vector<NodePtr> shortest_path_dag(Graph& graph, std::function<double(const NodePtr, const NodePtr)> cost_function, BaseContainer<NodePtr>& cont)
{
    // std::priority_queue<NodePtr, std::vector<NodePtr>, compareNodesFunction> Q;

    // auto d_fun = [](const NodePtr& a, const NodePtr& b) { return math::norm2Diff(a->data, b->data); };
    // auto d_fun = [](const NodePtr& a, const NodePtr& b) { return b->dist < a->dist; };

    // std::shared_ptr<BaseContainer<NodePtr>> Q;
    // Q = std::make_shared<StackContainer<NodePtr>>();
    // StackContainer<NodePtr> Q;
    // PriorityStackContainer<NodePtr> Q(graph.size(), d_fun);
    // OcplPriorityQueueContainer<NodePtr> Q(d_fun);

    const std::vector<NodePtr>& start_nodes = graph.getStartNodes();

    // Add the starting nodes to the queue.
    for (auto start_node : start_nodes)
    {
        start_node->dist = start_node->cost;
        start_node->visited = true;
        cont.push(start_node, 0);
    }

    // The actual graph search loop
    NodePtr current_node{ nullptr };
    bool goal_reached{ false };

    auto start = std::chrono::system_clock::now();
    std::chrono::seconds timeout(20);
    while (!cont.empty())
    {
        auto current = std::chrono::system_clock::now();
        if ((current - start) > timeout)
        {
            std::cout << "Maximum planning time reached.\n";
            break;
        }
        current_node = cont.pop();

        // A goal if found, when using a priority queue this should also be the goal
        // that gives the shortest path
        if (graph.isGoal(current_node))
        {
            goal_reached = true;
            break;
        }

        const std::vector<NodePtr>& neighbors = graph.getNeighbors(current_node);

        // the costs can be calculated in parallel
        // this can improve speed for complex cost functions
        for (auto nb : neighbors)
        {
            double dist_to_nb = cost_function(current_node, nb) + nb->cost;
            if (!std::isnan(dist_to_nb))
            {
                double new_dist = current_node->dist + dist_to_nb;
                if (new_dist < nb->dist)
                {
                    nb->dist = new_dist;
                    nb->parent = current_node;
                }

                if (!nb->visited)
                {
                    cont.push(nb, nb->waypoint_index);
                    nb->visited = true;
                }
            }
        }
    }

    if (!goal_reached)
    {
        std::cout << "None of the goals are reached in graph search.\n";
        return graph.extract_partial_solution();
    }

    assert(current_node != nullptr);

    return graph.extract_solution(current_node);
}

std::ostream& operator<<(std::ostream& os, const Node& node)
{
    os << "Node: (";
    for (const auto value : node.data)
    {
        os << value << ", ";
    }
    os << ")";
    return os;
}
}  // namespace ocpl
