#pragma once

#include <deque>
#include <functional>
#include <vector>
#include <iostream>

namespace planner
{
typedef std::vector<double> JointPositions;
typedef std::function<std::vector<JointPositions>(size_t waypoint, const JointPositions& q_bias)> LocalSampler;

std::ostream& operator<<(std::ostream& os, const JointPositions& q)
{
    for (auto qi : q)
        os << qi << ", ";
    return os;
}


struct Vertice
{
    JointPositions q;
    size_t waypoint;
    // double distance;
};

std::ostream& operator<<(std::ostream& os, const Vertice& v)
{
    os << "{ (";
    for (auto a : v.q)
    {
        os << a << ", ";
    }
    os << ") " << v.waypoint << " }";
    return os;
}


class BaseContainer
{
  public:
    virtual ~BaseContainer() = default;
    virtual Vertice pop() = 0;
    virtual void push(const Vertice& v) = 0;
    virtual bool empty() = 0;
};

class StackContainer : public BaseContainer
{
    std::vector<Vertice> data;

  public:
    virtual Vertice pop() override
    {
        Vertice v = data.back();
        data.pop_back();
        return v;
    }
    virtual void push(const Vertice& v) override
    {
        data.push_back(v);
    }
    virtual bool empty() override
    {
        return data.empty();
    }
};

class QueueContainer : public BaseContainer
{
    std::deque<Vertice> data;

  public:
    virtual Vertice pop() override
    {
        Vertice v = data.front();
        data.pop_front();
        return v;
    }
    virtual void push(const Vertice& v) override
    {
        data.push_back(v);
    }
    virtual bool empty() override
    {
        return data.empty();
    }
};

std::vector<JointPositions> search(const JointPositions& q_start, LocalSampler local_sampler, size_t num_points,
                                   BaseContainer& con)
{
    con.push(Vertice{ q_start, 0 });
    size_t k{ 1 }, k_prev{ 0 };
    Vertice current;

    std::vector<JointPositions> path;
    path.reserve(num_points);
    while (!con.empty())
    {
        current = con.pop();
        k = current.waypoint;

        std::cout << "Current v: " << current << "\n";
        if (k > k_prev)
        {
            path.push_back(current.q);
            if (k == num_points)
            {
                return path;  // Success!
            }
        }
        else
        {
            for (size_t i{0}; i < (k_prev - k + 1); ++i)
            {
                if (!path.empty())
                    path.pop_back();  // TODO
            }
            path.push_back(current.q);
        }

        for (auto q : local_sampler(k + 1, current.q))
        {
            con.push(Vertice{ q, k + 1 });
            std::cout << "Added vertice: " << q << " | " << k + 1 << "\n";
        }
        k_prev = k;
    }
    return {};  // Failure :(
}
}  // namespace planner
