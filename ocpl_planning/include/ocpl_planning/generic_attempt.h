#pragma once

#include <queue>
#include <deque>
#include <functional>
#include <vector>
#include <iostream>
#include <stdexcept>

#include <Eigen/Dense>

namespace planner
{
typedef Eigen::VectorXd JointPositions;

struct Vertice
{
    JointPositions q;
    size_t waypoint;
    double distance{ 0.0 };
};

typedef std::function<std::vector<JointPositions>(size_t waypoint, const JointPositions& q_bias)> LocalSampler;
typedef std::function<double(const JointPositions& from, const JointPositions& to)> DistanceMetric;
// typedef std::function<bool(const Vertice&, const Vertice&)> VerticeCompare;

// bool test(const Vertice&, const Vertice&);

// VPriorityQueue vp(test);

struct VerticeCompare
{
    bool operator()(const Vertice& a, const Vertice& b) const
    {
        return a.distance > b.distance;
    }
};

using VPriorityQueue = std::priority_queue<Vertice, std::deque<Vertice>, VerticeCompare>;

std::ostream& operator<<(std::ostream& os, const Vertice& v)
{
    os << "{ (" << v.q.transpose();
    os << ") " << v.waypoint << " d: " << v.distance << " }";
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
    StackContainer() = default;
    ~StackContainer() = default;
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
    QueueContainer() = default;
    ~QueueContainer() = default;
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

class PriorityQueueContainer : public BaseContainer
{
    size_t max_waypoints_;
    std::vector<VPriorityQueue> data_;

    size_t pop_waypoint_{ 0 };

  public:
    // PriorityQueueContainer(size_t max_waypoints, VerticeCompare compare_fun) : max_waypoints_(max_waypoints)
    // {
    //     for (size_t i{ 0 }; i < max_waypoints; ++i)
    //         data_.push_back(VPriorityQueue(compare_fun));
    // }

    PriorityQueueContainer(size_t max_waypoints) : max_waypoints_(max_waypoints), data_(max_waypoints)
    {
    }

    ~PriorityQueueContainer() = default;

    virtual Vertice pop() override
    {
        if (data_[pop_waypoint_].empty())
        {
            if (pop_waypoint_ < max_waypoints_ - 1)
            {
                pop_waypoint_++;
                return pop();
            }
            else
            {
                throw std::out_of_range("Trying to pop an element from an empty PriorityQueueContainer");
            }
        }
        else
        {
            Vertice v = data_[pop_waypoint_].top();
            data_[pop_waypoint_].pop();
            return v;
        }
    }
    virtual void push(const Vertice& v) override
    {
        if (v.waypoint < max_waypoints_)
        {
            data_[v.waypoint].push(v);
            // potentially move the pop waypoint back
            if (v.waypoint < pop_waypoint_)
            {
                pop_waypoint_ = v.waypoint;
            }
        }
        else
        {
            throw std::out_of_range("Trying to push an element past the waypoint size of this PriorityQueueContainer");
        }
    }
    virtual bool empty() override
    {
        return (pop_waypoint_ == (max_waypoints_ - 1) && data_[pop_waypoint_].empty());
    }
};

class PriorityStackContainer : public BaseContainer
{
    size_t max_waypoints_;
    std::vector<VPriorityQueue> data_;

    size_t current_waypoint_{ 0 };

  public:
    // PriorityStackContainer(size_t max_waypoints, VerticeCompare compare_fun) : max_waypoints_(max_waypoints)
    // {
    //     for (size_t i{ 0 }; i < max_waypoints; ++i)
    //         data_.push_back(VPriorityQueue(compare_fun));
    // }

    PriorityStackContainer(size_t max_waypoints) : max_waypoints_(max_waypoints), data_(max_waypoints)
    {
    }

    ~PriorityStackContainer() = default;

    virtual Vertice pop() override
    {
        if (data_[current_waypoint_].empty())
        {
            if (current_waypoint_ > 0)
            {
                current_waypoint_--;
                return pop();
            }
            else
            {
                throw std::out_of_range("Trying to pop an element from an empty PriorityQueueContainer");
            }
        }
        else
        {
            Vertice v = data_[current_waypoint_].top();
            data_[current_waypoint_].pop();
            return v;
        }
    }
    virtual void push(const Vertice& v) override
    {
        if (v.waypoint < max_waypoints_)
        {
            current_waypoint_ = v.waypoint;
            data_[current_waypoint_].push(v);
        }
        else
        {
            throw std::out_of_range("Trying to push an element past the waypoint size of this PriorityQueueContainer");
        }
    }
    virtual bool empty() override
    {
        return (current_waypoint_ == 0 && data_[current_waypoint_].empty());
    }
};

std::vector<JointPositions> search(const JointPositions& q_start, LocalSampler local_sampler, size_t num_points,
                                   BaseContainer& con, DistanceMetric distFun)
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
            if (k == (num_points - 1))
            {
                return path;  // Success!
            }
        }
        else
        {
            for (size_t i{ 0 }; i < (k_prev - k + 1); ++i)
            {
                if (path.empty())
                    break;
                else
                    path.pop_back();  // TODO
            }
            path.push_back(current.q);
        }

        for (auto q : local_sampler(k + 1, current.q))
        {
            con.push(Vertice{ q, k + 1, current.distance + distFun(current.q, q)});
            std::cout << "Added vertice: " << q << " | " << k + 1 << "\n";
        }
        k_prev = k;
    }
    return {};  // Failure :(
}
}  // namespace planner
