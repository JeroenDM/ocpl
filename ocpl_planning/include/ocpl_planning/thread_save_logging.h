#pragma once

#include <iostream>
#include <string>
#include <vector>

#include <ocpl_planning/types.h>

namespace ocpl
{
/** \brief Helper function to format vectors to pretty print settings. **/
std::string format(const std::vector<int>& v);

/** \brief Helper function to convert SamplerType to string for pretty printing below. **/
std::string format(const SamplerType& t);

/** \brief Pretty printing of planner settings. **/
std::ostream& operator<<(std::ostream& os, const PlannerSettings& ps);

/** \brief Write the path to a simple csv file. **/
void savePath(const std::string& filename, const std::vector<JointPositions>& path);

/** \brief Read a path that was saved as a numpy 2D array (num_pts x num_dof). **/
std::vector<JointPositions> loadPath(const std::string& filename);

}  // namespace ocpl
#pragma once

#include <mutex>

namespace ocpl
{
/** Quick and dirty attempt at thread save logging.
 *
 * Only a single instance can exist at the time (google how to do singleton in C++).
 * **/
class TSLogger
{
    std::mutex mutex_;

  public:
    TSLogger() = default;
    ~TSLogger() = default;

    template <typename T>
    void log(const T& message)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        std::cout << message;
    }

    void logWaypoint(size_t waypoint, size_t num_samples)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        std::cout << "ocpl_planning: processing path point " << waypoint;
        std::cout << " found " << num_samples << " samples\n";
    }

    void logWaypoint(size_t waypoint, size_t num_samples, size_t iters)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        std::cout << "ocpl_planning: processing path point " << waypoint;
        std::cout << " found " << num_samples << " samples ";
        std::cout << iters << " iters\n";
    }
};
}  // namespace ocpl
