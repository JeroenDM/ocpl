#include <ros/ros.h>

#include <algorithm>
#include <limits>
#include <chrono>
#include <map>

#include <ocpl_ros/moveit_robot_examples.h>
#include <ocpl_ros/rviz.h>
#include <ocpl_sampling/grid_sampler.h>
#include <ocpl_sampling/halton_sampler.h>
#include <ocpl_sampling/random_sampler.h>
#include <ocpl_tsr/task_space_regions.h>

#include <ocpl_planning/planners.h>
#include <ocpl_planning/factories.h>
#include <ocpl_planning/cost_functions.h>

#include <ompl/datastructures/NearestNeighborsLinear.h>

using namespace ocpl;
using Eigen::AngleAxisd;
using Eigen::Isometry3d;
using Eigen::Translation3d;
using Eigen::Vector3d;

void sleep(double time)
{
    ros::Duration(time).sleep();
};

inline double deg2rad(double deg)
{
    return deg * M_PI / 180.0;
}

std::vector<JointPositions> interpolate(MoveItRobot& robot, const JointPositions& from, const JointPositions to)
{
    assert(from.size() == to.size());

    static constexpr double MAX_J_STEP{ 0.1 };
    static constexpr int MIN_NUM_STEPS{ 5 };
    const std::size_t DOF{ from.size() };

    // calculate the number of interpolation points
    double j_step{ 0.0 };
    for (std::size_t i{ 0 }; i < DOF; ++i)
    {
        j_step = std::max(j_step, std::abs(to[i] - from[i]));
    }
    int num_steps = std::max((int)std::ceil(j_step / MAX_J_STEP), MIN_NUM_STEPS);

    // construct the interpolated path
    double ds = 1.0 / (num_steps + 1);
    std::vector<JointPositions> path;
    for (double s{ 0.0 }; s <= 1; s += ds)
    {
        JointPositions q(DOF);
        for (std::size_t i{ 0 }; i < DOF; ++i)
            q[i] = from[i] + s * (to[i] - from[i]);
        path.push_back(q);
    }
    return path;
}

std::vector<JointPositions> sampleGoal(IndustrialRobot& robot, Rviz& rviz)
{
    Transform tf = Translation3d(2.95, 1.4, 0.9527) * AngleAxisd(0.0, Vector3d::UnitX()) *
                   AngleAxisd(deg2rad(135), Vector3d::UnitY()) * AngleAxisd(deg2rad(90), Vector3d::UnitZ());

    rviz.plotPose(tf);

    static SamplerPtr redundant_joint_sampler =
        createIncrementalSampler({ robot.getJointPositionLimits().at(0) }, SamplerType::RANDOM);

    std::vector<JointPositions> valid_samples;
    for (auto q_red : redundant_joint_sampler->getSamples(20))
    {
        auto ik_sol = robot.ik(tf, q_red);

        if (ik_sol.size() == 0)
            std::cout << "Failed to find ik solutions. \n";

        for (JointPositions q : ik_sol)
        {
            if (!robot.isInCollision(q))
            {
                valid_samples.push_back(q);
                robot.plot(rviz.visual_tools_, q, rviz_visual_tools::GREEN);
                sleep(0.1);
            }
            else
            {
                robot.plot(rviz.visual_tools_, q, rviz_visual_tools::RED);
                sleep(0.1);
            }
        }
    }
    return valid_samples;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ocpl_moveit_demo_rn");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    IndustrialRobot robot("tool_tip");
    Rviz rviz;
    rviz.clear();

    JointPositions q_home{ 0.1, 0, -1.5708, 1.5708, 0, 0, 0 };

    robot.plot(rviz.visual_tools_, q_home);
    sleep(0.5);

    const size_t GRAPH_SIZE{300};
    const size_t TRY_CONNECT{50};

    auto sampler = createIncrementalSampler(robot.getJointPositionLimits(), SamplerType::RANDOM);
    auto valid_state_sampler = [&robot, sampler](int num_samples) {
        std::vector<JointPositions> samples = sampler->getSamples(num_samples);
        std::vector<JointPositions> valid_samples;
        valid_samples.reserve(samples.size());
        for (auto q : samples)
        {
            if (!robot.isInCollision(q))
                valid_samples.emplace_back(q);
        }
        valid_samples.shrink_to_fit();
        return valid_samples;
    };

    auto check_local_path = [&robot](const JointPositions& from, const JointPositions to) {
        assert(from.size() == to.size());

        static constexpr double MAX_J_STEP{ 0.1 };
        static constexpr int MIN_NUM_STEPS{ 5 };
        const std::size_t DOF{ from.size() };

        // calculate the number of interpolation points
        double j_step{ 0.0 };
        for (std::size_t i{ 0 }; i < DOF; ++i)
        {
            j_step = std::max(j_step, std::abs(to[i] - from[i]));
        }
        int num_steps = std::max((int)std::ceil(j_step / MAX_J_STEP), MIN_NUM_STEPS);

        // check collision along interpolated path
        // we do not check from or to assuming they are already valid
        double ds = 1.0 / (num_steps + 1);
        JointPositions q(DOF);
        for (double s{ ds }; s < 1; s += ds)
        {
            for (std::size_t i{ 0 }; i < DOF; ++i)
                q[i] = from[i] + s * (to[i] - from[i]);
            if (robot.isInCollision(q))
                return false;
        }
        return true;
    };

    std::vector<JointPositions> goal_samples = sampleGoal(robot, rviz);
    std::vector<NodePtr> goal_nodes;
    for (auto goal : goal_samples)
    {
        goal_nodes.push_back(std::make_shared<Node>(goal, 0.0));
    }
    if (goal_nodes.size() == 0)
    {
        std::cout << "No valid goal states found.\n";
        exit(1);
    }

    ompl::NearestNeighborsLinear<NodePtr> graph;
    graph.setDistanceFunction(L1NormDiff);

    auto some_samples = valid_state_sampler(GRAPH_SIZE);
    std::vector<NodePtr> nodes(some_samples.size());
    for (size_t i{ 0 }; i < some_samples.size(); ++i)
        nodes[i] = std::make_shared<Node>(some_samples[i], 0.0);

    graph.add(nodes);

    // try to connect samples in a graph
    // setup a function to get a random index into the graph
    RandomSampler s;
    s.addDimension(0, some_samples.size());
    auto getRandomIndex = [&s]() { return (int)(s.getSamples(1)[0][0]); };

    Tree tree;
    for (NodePtr current : nodes)
    {
        std::vector<NodePtr> nb;
        nb.reserve(TRY_CONNECT);
        graph.nearestK(current, TRY_CONNECT, nb);
        for (auto neighbour : nb)
        {
            if (check_local_path(current->data, neighbour->data))
            {
                double cost = L1NormDiff(current, neighbour);
                tree[current].push_back(Edge{ neighbour, cost });
                tree[neighbour].push_back(Edge{ current, cost });
            }
        }
        std::cout << "Found " << tree[current].size() << " connections.\n";
    }

    std::vector<NodePtr> start_nodes;
    start_nodes.push_back(std::make_shared<Node>(q_home, 0.0));
    std::vector<NodePtr> nb;
    nb.reserve(TRY_CONNECT);
    graph.nearestK(start_nodes[0], TRY_CONNECT, nb);
    for (auto neighbour : nb)
    {
        if (check_local_path(start_nodes[0]->data, neighbour->data))
        {
            tree[start_nodes[0]].push_back(Edge{ neighbour, L1NormDiff(start_nodes[0], neighbour) });
        }
    }
    std::cout << "Found " << tree[start_nodes[0]].size() << " connections to start.\n";

    for (auto goal : goal_nodes)
    {
        std::vector<NodePtr> nb;
        nb.reserve(TRY_CONNECT);
        graph.nearestK(goal, TRY_CONNECT, nb);
        for (auto neighbour : nb)
        {
            if (check_local_path(goal->data, neighbour->data))
            {
                double cost = L1NormDiff(goal, neighbour);
                tree[goal].push_back(Edge{ neighbour, cost});
                tree[neighbour].push_back(Edge{ goal, cost });
            }
        }
        std::cout << "Found " << tree[goal].size() << " connections to goal.\n";
    }

    auto solution = shortest_path(tree, start_nodes, goal_nodes);

    std::cout << "Found a path of length: " << solution.size() << "\n";
    std::vector<JointPositions> path;
    if (solution.size() > 1)
    {
        path.push_back(solution[0]->data);
        for (size_t i{ 1 }; i < solution.size(); ++i)
        {
            auto segment = interpolate(robot, solution[i - 1]->data, solution[i]->data);
            for (size_t i{ 1 }; i < segment.size(); ++i)
                path.push_back(segment[i]);
        }
        for (auto q : path)
        {
            robot.plot(rviz.visual_tools_, q);
            sleep(0.1);
        }
    }

    return 0;
}
