#pragma once

#include <chrono>

namespace ocpl
{
/** \brief A simple stopwatch that uses std::chrono. **/
class Timer
{
    std::chrono::steady_clock::time_point start_time_;

  public:
    void start();
    double stop();
};
}  // namespace ocpl
