#include <ocpl_graph/tree.h>

#include <algorithm>
#include <queue>
#include <iostream>
#include <cmath>  // isnan
#include <cassert>

namespace ocpl
{
struct compareNodesFunction
{
    bool operator()(NodePtr n1, NodePtr n2) const
    {
        return (*n2).dist < (*n1).dist;
    }
};

std::vector<NodePtr> _extract_path(NodePtr goal)
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

std::vector<NodePtr> shortest_path_dag(const std::vector<std::vector<NodePtr>>& nodes,
                                       std::function<double(const NodePtr, const NodePtr)> cost_function)
{
    std::priority_queue<NodePtr, std::vector<NodePtr>, compareNodesFunction> Q;

    const std::vector<NodePtr>& start_nodes = nodes.front();
    const std::vector<NodePtr>& goal_nodes = nodes.back();

    // Give the nodes correct waypoint indices to do fast nearest neighbor search.
    for (std::size_t index{ 0 }; index < nodes.size(); ++index)
    {
        for (auto& n : nodes[index])
            n->waypoint_index = index;
    }

    // Add the starting nodes to the queue.
    for (auto start_node : start_nodes)
    {
        start_node->dist = start_node->cost;
        start_node->visited = true;
        Q.push(start_node);
    }

    // The actual graph search loop
    NodePtr current_node{ nullptr };
    bool goal_reached{ false };

    // TODO replace this with for loop over samples and remove priority queue for DAG case
    while (!Q.empty())
    {
        current_node = Q.top();
        Q.pop();

        // A goal if found, when using a priority queue this should also be the goal
        // that gives the shortest path
        if (current_node->waypoint_index == (nodes.size() - 1))
        {
            goal_reached = true;
            break;
        }

        const std::vector<NodePtr>& neighbors = nodes.at(current_node->waypoint_index + 1);

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
                    Q.push(nb);
                    nb->visited = true;
                }
            }
        }
    }

    if (!goal_reached)
    {
        std::cout << "None of the goals are reached in graph search.\n";
        return _extract_partial_solution(nodes);
    }

    assert(current_node != nullptr);

    return _extract_path(current_node);
}

std::vector<NodePtr> _extract_partial_solution(const std::vector<std::vector<NodePtr>>& nodes)
{
    for (std::size_t pi{ nodes.size() - 1 }; pi >= 0; --pi)
    {
        auto pt_nodes = nodes[pi];
        auto node_iter = std::min_element(pt_nodes.begin(), pt_nodes.end(),
                                          [](const NodePtr& a, const NodePtr& b) { return a->dist < b->dist; });
        if (node_iter != pt_nodes.end() && (*node_iter)->parent != nullptr)
        {
            std::cout << "Found partial path up until pt index: " << pi << ".\n";
            return _extract_path(*node_iter);
        }
    }
    std::cout << "Also could not extract partial solution.\n";
    return {};
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

std::ostream& operator<<(std::ostream& os, const Edge& edge)
{
    os << "Edge: " << *edge.child << " : " << edge.cost;
    return os;
}

std::ostream& operator<<(std::ostream& os, const std::vector<Edge>& edges)
{
    for (const auto& edge : edges)
    {
        os << edge << ", ";
    }
    return os;
}

}  // namespace ocpl
