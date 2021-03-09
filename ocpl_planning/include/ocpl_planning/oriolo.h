#pragma once

#include <functional>
#include <vector>
#include <queue>
#include <cmath>

// graph includes
#include <unordered_map>
#include <limits>
#include <memory>
#include <exception>

#include <ocpl_tsr/task_space_regions.h>
#include <ocpl_sampling/sampler.h>
#include <ocpl_planning/planner_base.h>
#include <mutex>

namespace ocpl
{
/** This planner uses it's own graph implementation.
 *
 * Defined in a namespace to avoid name collision with ocpl::Node
 * from ocpl_graph used in ocpl_planners included above.
 *
 * **/
namespace graph
{
struct NodeData
{
    std::vector<double> q;
    size_t waypoint;
};

struct Node
{
    NodeData data;
    std::shared_ptr<Node> parent{ nullptr };
    bool visited{ false };
    double distance{ std::numeric_limits<double>::max() };

    Node(const NodeData& d) : data(d)
    {
    }
};

using NodePtr = std::shared_ptr<Node>;
using Tree = std::unordered_map<std::shared_ptr<Node>, std::vector<NodePtr>>;

}  // namespace graph

namespace oriolo
{
// struct OrioloSpecificSettings
// {
//     std::string METHOD{ "greedy" };
//     // d: "maximum allowed displacement of a single joint."
//     double D{ 0.4 };
//     // upperbound on calls to randConf to find joint positions for a waypoint along thepath
//     size_t MAX_SHOTS{ 100 };
//     // maximum iterations to find a good start configuration for greedy search
//     size_t MAX_ITER{ 100 };
//     // how many times do extend before adding another start config to the tree
//     size_t MAX_EXTEND{ 50000 };
//     std::string name{};
// };

/** Planner from Orioli paper.
 *
 * Parameters have different names:
 * MAX_SHOTS, MAX_EXTEND = max_iters
 * D = cspace_delta
 *
 * D: "maximum allowed displacement of a single joint."
 * MAX_SHOTS: upperbound on calls to randConf to find joint positions for a waypoint along thepath
 * MAX_ITER maximum iterations to find a good start configuration for greedy search
 * MAX_EXTEND how many times do extend before adding another start config to the tree
 *
 * **/
class OrioloPlanner : public Planner
{
  private:
    double EXTEND_STEP_{ 0.0 };

    graph::Tree tree_;

    /******************************
     * BUILDING BLOCKS
     * ****************************/
    /** \brief Sample with the values for the redundant joints already specified.
     * 
     * (Used in the rrtlike planner for the extend method.)
     * **/
    JointPositions sampleIK(const TSR& tsr, const JointPositions& q_red, const JointPositions& q_bias);

    std::vector<JointPositions> step(size_t start_index, size_t stop_index, const JointPositions& q_start,
                                     const std::vector<TSR>& task);
    std::pair<bool, graph::NodeData> extend(const std::vector<TSR>& task, graph::Tree& tree);
    graph::NodePtr getNear(const JointPositions& q, graph::Tree& tree);

    /******************************
     * PLANNING ALGORITHMS
     * ****************************/
    std::vector<JointPositions> greedy(const std::vector<TSR>& task);
    std::vector<JointPositions> bidirectionalGreedy(const std::vector<TSR>& task);
    std::vector<JointPositions> rrtLike(const std::vector<TSR>& task);

  public:
    OrioloPlanner(const Robot& robot, const PlannerSettings& settings)
      : Planner(robot, settings), EXTEND_STEP_(settings.cspace_delta * std::sqrt(robot.num_dof))
    {
    }

    void changeSettings(const PlannerSettings& new_settings) override
    {
        Planner::changeSettings(new_settings);
        EXTEND_STEP_ = new_settings.cspace_delta * std::sqrt(robot_.num_dof);
    }

    Solution solve(const std::vector<TSR>& task) override;
    Solution solve(const std::vector<TSR>& task,
                   std::function<double(const JointPositions&, const JointPositions&)> path_cost_fun,
                   std::function<double(const TSR&, const JointPositions&)> state_cost_fun) override;
};

}  // namespace oriolo
}  // namespace ocpl
