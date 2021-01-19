#include <ros/ros.h>

#include <algorithm>
#include <limits>
#include <chrono>
#include <map>
#include <functional>

#include <ocpl_ros/moveit_robot_examples.h>
#include <ocpl_ros/rviz.h>
#include <ocpl_sampling/grid_sampler.h>
#include <ocpl_sampling/halton_sampler.h>
#include <ocpl_sampling/random_sampler.h>
#include <ocpl_tsr/task_space_regions.h>
#include <ocpl_planning/prm.h>

#include <ocpl_planning/planners.h>
#include <ocpl_planning/factories.h>
#include <ocpl_planning/cost_functions.h>

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

    auto check_local_path = [&robot](const JointPositions& from, const JointPositions& to) {
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

    auto sampler = createIncrementalSampler(robot.getJointPositionLimits(), SamplerType::RANDOM);
    auto valid_state_sampler = [&robot, sampler](int num_samples) {
        std::vector<JointPositions> samples = sampler->getSamples(num_samples);
        std::vector<JointPositions> valid_samples;
        valid_samples.reserve(samples.size());
        for (auto q : samples)
        {
            if (!robot.isInCollision(q))
                valid_samples.push_back(q);
        }
        valid_samples.shrink_to_fit();
        return valid_samples;
    };

    SamplerPtr redundant_joint_sampler =
        createIncrementalSampler({ robot.getJointPositionLimits().at(0) }, SamplerType::RANDOM);
    auto goal_sampler = [&robot, redundant_joint_sampler](int num_samples) {
        Transform tf = Translation3d(2.95, 1.4, 0.9527) * AngleAxisd(0.0, Vector3d::UnitX()) *
                       AngleAxisd(deg2rad(135), Vector3d::UnitY()) * AngleAxisd(deg2rad(90), Vector3d::UnitZ());

        std::vector<JointPositions> valid_samples;
        for (auto q_red : redundant_joint_sampler->getSamples(num_samples))
        {
            auto ik_sol = robot.ik(tf, q_red);
            for (JointPositions q : ik_sol)
            {
                if (!robot.isInCollision(q))

                    valid_samples.push_back(q);
            }
        }
        return valid_samples;
    };

    auto start_sampler = [q_home](int /* num_samples*/) {
        IKSolution samples;
        samples.push_back(q_home);
        return samples;
    };

    Roadmap roadmap = createRoadmap(valid_state_sampler, start_sampler, goal_sampler, { 300, 20, 20 });
    Tree tree = connectRoadmap(roadmap, check_local_path, L1NormDiff2);

    auto solution = shortest_path(tree, roadmap.start_nodes, roadmap.goal_nodes);

    std::cout << "Found a path of length: " << solution.size() << "\n";
    std::vector<JointPositions> path;
    if (solution.size() > 1)
    {
        path.push_back(solution[0]->data);
        for (size_t i{ 1 }; i < solution.size(); ++i)
        {
            auto segment = interpolate(solution[i - 1]->data, solution[i]->data);
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
