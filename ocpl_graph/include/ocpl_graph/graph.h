#pragma once

#include <ocpl_graph/containers.h>

#include <functional>
#include <limits>
#include <memory>
#include <vector>

namespace ocpl
{
namespace tree
{
constexpr double INF{ std::numeric_limits<double>::max() };

template <typename Data>
struct Node;

template <typename Data>
struct Node
{
    Data data;
    size_t waypoint;
    double distance{ INF };
    bool visited{ false };
    std::shared_ptr<Node<Data>> parent{ nullptr };

    Node(const Data& d, const size_t wp) : data(d), waypoint(wp)
    {
    }
};

template <typename Data>
using NodePtr = std::shared_ptr<Node<Data>>;

template <typename NodeType>
using get_neighbors_t = std::function<std::vector<NodeType>(const NodeType&, const size_t)>;

template <typename NodeType>
using distance_t = std::function<double(const NodeType&, const NodeType&)>;

template <typename Data>
void search(BaseContainer<NodePtr<Data>>& container, get_neighbors_t<NodePtr<Data>> get_neighbors_fun,
            std::vector<NodePtr<Data>> start_nodes, distance_t<NodePtr<Data>> distance_fun)
{
    for (NodePtr<Data>& node : start_nodes)
    {
        container.push(node, 0);
        node->distance = 0.0;
    }

    size_t k{ 0 };
    NodePtr<Data> current;
    while (!container.empty())
    {
        current = container.pop();

        for (NodePtr<Data>& nb : get_neighbors_fun(current, k))
        {
            if (!nb.visited)
            {
                container.push(nb);
            }
            else
            {
                auto new_d = current->distance + distance_fun(current, nb);
                if (new_d < nb.distance)
                {
                    nb->distance = new_d;
                    nb->parent = current;
                }
            }
        }
    }
}

// template <typename T>
// std::vector<NodePtr<T>> treeSearch();
}  // namespace tree
namespace graph
{
}
}  // namespace ocpl
