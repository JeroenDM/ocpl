#pragma once

#include <functional>
#include <vector>
#include <queue>

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

  public:
    OrioloPlanner(const Robot& robot, const PlannerSettings& settings) : Planner(robot, settings)
    {
    }
    Solution solve(const std::vector<TSR>& task) override;
    Solution solve(const std::vector<TSR>& task,
                   std::function<double(const JointPositions&, const JointPositions&)> path_cost_fun,
                   std::function<double(const TSR&, const JointPositions&)> state_cost_fun) override;

    /** \brief Unbiased inverse kinematics for random samle in task space regions. **/
    JointPositions invKin(const TSR& tsr, const JointPositions& q_red);

    /** \brief Inverse kinematics with a bias value to stay close to.
     *
     * Returns empty vector if it failed.
     * **/
    JointPositions invKin(const TSR& tsr, const JointPositions& q_red, const JointPositions& q_bias);

    /** \brief return random positions for redundant joints. Centered around a bias position.
     *
     * In paper it is described as "generated through a limited random perturbation of q_red_bias".
     * I interpret limited as using the maximum displacement parameter d;
     * **/
    JointPositions randRed(const JointPositions& q_bias);
    JointPositions randRed();

    // all variations on randConf to get random joint positions
    JointPositions randConf();
    JointPositions randConf(const TSR& tsr);
    JointPositions randConf(const TSR& tsr, const JointPositions& q_bias);

    // state and path validation (collision checking)
    bool noColl(const JointPositions& q);
    bool noColl(const JointPositions& q_from, const JointPositions& q_to);

    std::vector<JointPositions> step(size_t start_index, size_t stop_index, const JointPositions& q_start,
                                     const std::vector<TSR>& task);
    std::vector<JointPositions> greedy(const std::vector<TSR>& task);
    std::vector<JointPositions> bidirectionalGreedy(const std::vector<TSR>& task);

    std::pair<bool, graph::NodeData> extend(const std::vector<TSR>& task, graph::Tree& tree);

    graph::NodePtr getNear(const JointPositions& q, graph::Tree& tree);

    std::vector<JointPositions> rrtLike(const std::vector<TSR>& task);

    /** \brief Get a (random) robot configurations for a given waypoint along the path. **/
    JointPositions sample(const TSR& tsr);

    /** \brief Get a (random) biased robot configurations for a given waypoint along the path.
     *
     * The solution space for the given waypoint is sampled in a regions around the q_bias;
     *
     * **/
    JointPositions sample(const TSR& tsr, const JointPositions& q_bias);

    // std::vector<JointPositions> greedy2(const std::vector<TSR>& task);

    //     PriorityQueue getLocalSamples(const TSR& tsr, const JointPositions& q_bias, int num_samples);

    // JointPositions findChildren(const TSR& tsr, const JointPositions& q_bias, int num_samples = 1);
};

}  // namespace oriolo
}  // namespace ocpl
