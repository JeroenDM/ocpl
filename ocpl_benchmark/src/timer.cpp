#include "ocpl_benchmark/timer.h"

#include <chrono>

namespace ocpl
{
void Timer::start()
{
    start_time_ = std::chrono::steady_clock::now();
}

double Timer::stop()
{
    auto stop_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> t = stop_time - start_time_;
    return t.count();
}
}  // namespace ocpl
