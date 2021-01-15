#include <ocpl_graph/tree.h>

#include <queue>
#include <algorithm>  // std::find, std::min_element
#include <iostream>
#include <cmath>  // isnan

namespace ocpl
{
void Node::reset()
{
    dist = std::numeric_limits<double>::max();
    parent = nullptr;
    visited = false;
}

struct compareNodesFunction
{
    bool operator()(NodePtr n1, NodePtr n2) const
    {
        return (*n2).dist < (*n1).dist;
    }
};

std::vector<NodePtr> _extract_path(const Tree& tree, NodePtr goal)
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

std::vector<NodePtr> shortest_path(Tree& tree, NodePtr start, NodePtr goal)
{
    //  std::queue<Node*> Q;
    std::priority_queue<NodePtr, std::vector<NodePtr>, compareNodesFunction> Q;

    start->dist = start->cost;

    Q.push(start);

    NodePtr current_node{ nullptr };
    std::vector<Edge> nb;

    while (!Q.empty())
    {
        //    current_node = Q.front();
        current_node = Q.top();
        Q.pop();

        if (current_node == goal)
        {
            break;
        }

        if (tree.find(current_node) == tree.end())
        {
            continue;
        }
        else
        {
            nb = tree[current_node];
        }

        for (auto edge : nb)
        {
            if (current_node->dist + edge.cost + edge.child->cost < edge.child->dist)
            {
                edge.child->dist = current_node->dist + edge.cost + edge.child->cost;
                edge.child->parent = current_node;
            }

            if (!edge.child->visited)
            {
                Q.push(edge.child);
                edge.child->visited = true;
            }
        }
    }

    return _extract_path(tree, current_node);
}

std::vector<NodePtr> shortest_path(Tree& tree, std::vector<NodePtr> start_nodes, std::vector<NodePtr> goal_nodes)
{
    std::priority_queue<NodePtr, std::vector<NodePtr>, compareNodesFunction> Q;

    for (auto start_node : start_nodes)
    {
        start_node->dist = start_node->cost;
        Q.push(start_node);
    }
    NodePtr current_node{ nullptr };
    std::vector<Edge> nb;

    bool goal_reached{ false };

    while (!Q.empty())
    {
        //    current_node = Q.front();
        current_node = Q.top();
        Q.pop();

        // is the current node a goal node?
        if (std::find(goal_nodes.begin(), goal_nodes.end(), current_node) != goal_nodes.end())
        // if (current_node == dummy_goal)
        {
            goal_reached = true;
            break;
        }

        if (tree.find(current_node) == tree.end())
        {
            continue;
        }
        else
        {
            nb = tree[current_node];
        }

        for (auto edge : nb)
        {
            if (current_node->dist + edge.cost + edge.child->cost < edge.child->dist)
            {
                edge.child->dist = current_node->dist + edge.cost + edge.child->cost;
                edge.child->parent = current_node;
            }

            if (!edge.child->visited)
            {
                Q.push(edge.child);
                edge.child->visited = true;
            }
        }
    }

    if (goal_reached)
    {
        std::cout << "Success! Graph search reached a goal node.\n";
    }
    else
    {
        std::cout << "Graph search did not reach a goal node!\n";
    }

    return _extract_path(tree, current_node);
}

std::vector<NodePtr> shortest_path_dag(const std::vector<std::vector<NodePtr>>& nodes,
                                       std::function<double(const NodePtr, const NodePtr)> cost_function)
{
    std::priority_queue<NodePtr, std::vector<NodePtr>, compareNodesFunction> Q;

    const std::vector<NodePtr>& start_nodes = nodes.front();
    const std::vector<NodePtr>& goal_nodes = nodes.back();

    // We keep track of how many goal nodes we have yet to reach.
    std::size_t goal_nodes_to_reach{ goal_nodes.size() };

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

    // TODO replace this with for loop over samples and remove priority queue for DAG case
    while (!Q.empty())
    {
        //    current_node = Q.front();
        current_node = Q.top();
        Q.pop();

        // std::cout << "graph search: (" << current_node->waypoint_index << " ) " << (*current_node) << "\n";
        // std::cout << "graph search: gntr " << goal_nodes_to_reach << "\n";

        // is the current node a goal node?
        // if (std::find(goal_nodes.begin(), goal_nodes.end(), current_node) != goal_nodes.end())
        if (current_node->waypoint_index == (nodes.size() - 1))
        {
            // keep going until all nodes are expanded!
            if (goal_nodes_to_reach > 0)
            {
                goal_nodes_to_reach--;
                continue;
            }
            else
            {
                // std::cout << "Found the last goal!\n";
                break;
            }
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

    if (goal_nodes_to_reach > 0)
    {
        std::cout << "Not all goal nodes reached.\n";
    }
    if (goal_nodes_to_reach == goal_nodes.size())
    {
        std::cout << "None of the goals are reached in graph search.\n";
        return _extract_partial_solution(nodes);
    }

    // find the goal node with the smallest distance
    auto node_iter = std::min_element(goal_nodes.begin(), goal_nodes.end(),
                                      [](const NodePtr& a, const NodePtr& b) { return a->dist < b->dist; });

    if (node_iter == goal_nodes.end())
    {
        std::cout << "None of the goals are reached.\n";
        return {};
    }

    return _extract_path(*node_iter);
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
