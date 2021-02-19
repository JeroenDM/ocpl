#pragma once

#include <vector>
#include <queue>

#include <ocpl_planning/types.h>

namespace ocpl
{
/** \brief Generic interface for a set of verticePtrs.
 *
 * Create a common interfaces for different types of verticePtr containers,
 * independent of how the underlying standard library container works. *
 * **/
class BaseContainer
{
  public:
    virtual ~BaseContainer() = default;
    virtual VerticePtr pop() = 0;
    virtual void push(const VerticePtr& v) = 0;
    virtual bool empty() = 0;
};

/** \brief Last in first out container. **/
class StackContainer : public BaseContainer
{
    std::vector<VerticePtr> data;

  public:
    StackContainer() = default;
    ~StackContainer() = default;
    virtual VerticePtr pop() override
    {
        VerticePtr v = data.back();
        data.pop_back();
        return v;
    }
    virtual void push(const VerticePtr& v) override
    {
        data.push_back(v);
    }
    virtual bool empty() override
    {
        return data.empty();
    }
};

struct VerticePtrCompare
{
    bool operator()(const VerticePtr& a, const VerticePtr& b) const
    {
        return a->distance > b->distance;
    }
};

using VPriorityQueue = std::priority_queue<VerticePtr, std::deque<VerticePtr>, VerticePtrCompare>;

class PriorityStackContainer : public BaseContainer
{
    size_t max_waypoints_;
    std::vector<VPriorityQueue> data_;

    size_t current_waypoint_{ 0 };

  public:
    // PriorityStackContainer(size_t max_waypoints, VerticePtrCompare compare_fun) : max_waypoints_(max_waypoints)
    // {
    //     for (size_t i{ 0 }; i < max_waypoints; ++i)
    //         data_.push_back(VPriorityQueue(compare_fun));
    // }

    PriorityStackContainer(size_t max_waypoints) : max_waypoints_(max_waypoints), data_(max_waypoints)
    {
    }

    ~PriorityStackContainer() = default;

    virtual VerticePtr pop() override
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
                throw std::out_of_range("Trying to pop an element from an empty PriorityStackContainer");
            }
        }
        else
        {
            VerticePtr v = data_[current_waypoint_].top();
            data_[current_waypoint_].pop();
            return v;
        }
    }
    virtual void push(const VerticePtr& v) override
    {
        if (v->waypoint < max_waypoints_)
        {
            current_waypoint_ = v->waypoint;
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
}  // namespace ocpl
