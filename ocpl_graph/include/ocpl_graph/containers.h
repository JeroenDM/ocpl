#pragma once

#include <algorithm>
#include <queue>
#include <vector>
#include <functional>

namespace ocpl
{
/** \brief Generic interface around containers in standard library. **/
template <typename T>
class BaseContainer
{
  public:
    virtual ~BaseContainer() = default;
    virtual T pop() = 0;
    virtual void push(const T& v, const size_t waypoint) = 0;
    virtual bool empty() = 0;
};

/** \brief Last in first out container. **/
template <typename T>
class StackContainer : public BaseContainer<T>
{
    std::vector<T> data;

  public:
    StackContainer() = default;
    ~StackContainer() = default;
    virtual T pop() override
    {
        T v = data.back();
        data.pop_back();
        return v;
    }
    virtual void push(const T& v, const size_t /* waypoint */) override
    {
        data.emplace_back(v);
    }
    virtual bool empty() override
    {
        return data.empty();
    }
};

/** \brief First in first out container. **/
template <typename T>
class QueueContainer : public BaseContainer<T>
{
    std::deque<T> data;

  public:
    QueueContainer() = default;
    ~QueueContainer() = default;
    virtual T pop() override
    {
        T v = data.front();
        data.pop_front();
        return v;
    }
    virtual void push(const T& v, const size_t /* waypoint */) override
    {
        data.emplace_back(v);
    }
    virtual bool empty() override
    {
        return data.empty();
    }
};

template <typename T>
class PriorityStackContainer : public BaseContainer<T>
{
    using compare_fun_t = std::function<bool(const T&, const T&)>;
    using priority_queue_t = std::priority_queue<T, std::vector<T>, compare_fun_t>;

    size_t max_waypoints_;
    std::vector<priority_queue_t> data_;

    size_t current_waypoint_{ 0 };

  public:
    PriorityStackContainer(size_t max_waypoints, compare_fun_t compare_fun) : max_waypoints_(max_waypoints)
    {
        for (size_t k{ 0 }; k < max_waypoints_; ++k)
            data_.emplace_back(priority_queue_t{ compare_fun });
    }

    ~PriorityStackContainer() = default;

    virtual T pop() override
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
            T v = data_[current_waypoint_].top();
            data_[current_waypoint_].pop();
            return v;
        }
    }

    virtual void push(const T& vertice, const size_t waypoint) override
    {
        if (waypoint < max_waypoints_)
        {
            data_[waypoint].emplace(vertice);
            if (waypoint > current_waypoint_)
            {
                current_waypoint_ = waypoint;
            }
        }
        else
        {
            throw std::out_of_range("Trying to push an element past the waypoint size of this PriorityQueueContainer");
        }
    }

    virtual bool empty() override
    {
        // Everyting above the current waypoint is empty by definition, only check below
        return std::all_of(data_.begin(), data_.begin() + current_waypoint_ + 1, [](const auto& q) { return q.empty(); });
    }
};

template <typename T>
class PriorityQueueContainer : public BaseContainer<T>
{
    using compare_fun_t = std::function<bool(const T&, const T&)>;
    using priority_queue_t = std::priority_queue<T, std::vector<T>, compare_fun_t>;

    size_t max_waypoints_;
    std::vector<priority_queue_t> data_;

    size_t current_waypoint_{ 0 };

  public:
    PriorityQueueContainer(size_t max_waypoints, compare_fun_t compare_fun) : max_waypoints_(max_waypoints)
    {
        for (size_t k{ 0 }; k < max_waypoints_; ++k)
            data_.emplace_back(priority_queue_t{ compare_fun });
    }

    ~PriorityQueueContainer() = default;

    virtual T pop() override
    {
        if (data_[current_waypoint_].empty())
        {
            if (current_waypoint_ < max_waypoints_ - 1)
            {
                current_waypoint_++;
                return pop();
            }
            else
            {
                throw std::out_of_range("Trying to pop an element from an empty PriorityQueueContainer");
            }
        }
        else
        {
            T v = data_[current_waypoint_].top();
            data_[current_waypoint_].pop();
            return v;
        }
    }
    virtual void push(const T& v, const size_t waypoint) override
    {
        if (waypoint < max_waypoints_)
        {
            data_[waypoint].push(v);
            // potentially move the pop waypoint back
            if (waypoint < current_waypoint_)
            {
                current_waypoint_ = waypoint;
            }
        }
        else
        {
            throw std::out_of_range("Trying to push an element past the waypoint size of this PriorityQueueContainer");
        }
    }
    virtual bool empty() override
    {
        // Everyting below the current waypoint is empty by definition, only check above
        return std::all_of(data_.begin() + current_waypoint_, data_.end(), [](const auto& q) { return q.empty(); });
    }
};
}  // namespace ocpl
