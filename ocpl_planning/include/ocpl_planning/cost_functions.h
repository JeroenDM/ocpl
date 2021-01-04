#pragma once

#include <cmath>

#include <ocpl_graph/tree.h>

namespace ocpl
{
inline double L2NormDiff(NodePtr n1, NodePtr n2)
{
    double cost{};
    int s = n1->data.size();
    for (int i = 0; i < n1->data.size(); ++i)
    {
        cost += std::sqrt((n1->data[i] - n2->data[i]) * (n1->data[i] - n2->data[i]));
    }
    return cost;
}

inline double L1NormDiff(NodePtr n1, NodePtr n2)
{
    double cost{};
    int s = n1->data.size();
    for (int i = 0; i < n1->data.size(); ++i)
    {
        cost += std::abs(n1->data[i] - n2->data[i]);
    }
    // if (cost > 1.0)
    //     cost = std::numeric_limits<double>::max();
    return cost;
}

}  // namespace ocpl
