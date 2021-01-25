#pragma once

#include <cmath>
#include <vector>

#include <Eigen/Geometry>

#include <ocpl_graph/tree.h>

namespace ocpl
{
inline double L2NormDiff(NodePtr n1, NodePtr n2)
{
    double cost{0.0};
    for (int i = 0; i < n1->data.size(); ++i)
    {
        cost += std::sqrt((n1->data[i] - n2->data[i]) * (n1->data[i] - n2->data[i]));
    }
    return cost;
}

inline double L2NormDiff2(const std::vector<double>& n1, const std::vector<double>& n2)
{
    assert(n1.size() == n2.size());
    
    double cost{0.0};
    for (int i = 0; i < n1.size(); ++i)
    {
        cost += std::sqrt((n1[i] - n2[i]) * (n1[i] - n2[i]));
    }
    return cost;
}

inline double L1NormDiff(NodePtr n1, NodePtr n2)
{
    double cost{0.0};
    for (int i = 0; i < n1->data.size(); ++i)
    {
        cost += std::abs(n1->data[i] - n2->data[i]);
    }
    // if (cost > 1.0)
    //     cost = std::numeric_limits<double>::max();
    return cost;
}

inline double L1NormDiff2(const std::vector<double>& n1, const std::vector<double>& n2)
{
    assert(n1.size() == n2.size());

    double cost{0.0};
    for (int i = 0; i < n1.size(); ++i)
    {
        cost += std::abs(n1[i] - n2[i]);
    }
    // if (cost > 1.0)
    //     cost = std::numeric_limits<double>::max();
    return cost;
}

inline double LInfNormDiff2(const std::vector<double>& n1, const std::vector<double>& n2)
{
    assert(n1.size() == n2.size());

    double cost{0.0};
    for (int i = 0; i < n1.size(); ++i)
    {
        cost  = std::max(std::abs(n1[i] - n2[i]), cost);
    }
    // if (cost > 1.0)
    //     cost = std::numeric_limits<double>::max();
    return cost;
}

inline double zeroStateCost(const TSR& /* tsr */, const std::vector<double>& /* q */)
{
    return 0.0;
};

}  // namespace ocpl
