#pragma once

#include <vector>

namespace ocpl
{
/** \brief Simple interpolation between two vectors. Returns one vector.**/
template <typename T>
inline std::vector<T> interpolate(std::vector<T> q_from, std::vector<T> q_to, T s)
{
    std::vector<T> q(q_from.size());
    for (std::size_t i = 0; i < q_from.size(); ++i)
    {
        q[i] = (1 - s) * q_from[i] + s * q_to[i];
    }
    return q;
}

inline double norm1Diff(const std::vector<double>& n1, const std::vector<double>& n2)
{
    assert(n1.size() == n2.size());

    double cost{ 0.0 };
    for (int i = 0; i < n1.size(); ++i)
    {
        cost += std::abs(n1[i] - n2[i]);
    }
    return cost;
}

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

inline double normInfDiff(const std::vector<double>& n1, const std::vector<double>& n2)
{
    assert(n1.size() == n2.size());

    double cost{ 0.0 };
    for (int i = 0; i < n1.size(); ++i)
    {
        cost = std::max(std::abs(n1[i] - n2[i]), cost);
    }
    // if (cost > 1.0)
    //     cost = std::numeric_limits<double>::max();
    return cost;
}

/** \Brief Return a value clipped to an interval if it fall's outside this range. **/
template <typename Scalar>
Scalar clip(Scalar value, Scalar lower, Scalar upper)
{
    return std::min(std::max(lower, value), upper);
}

inline double calculatePathCost(const std::vector<std::vector<double>>& path)
{
    double cost{ 0 };
    for (size_t i = 1; i < path.size(); i++)
    {
        cost += norm2Diff(path[i], path[i - 1]);
    }
    return cost;
}

}  // namespace ocpl
