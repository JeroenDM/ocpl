#include <ros/ros.h>

#include <algorithm>

#include <ocpl_ros/moveit_robot_examples.h>
#include <ocpl_ros/rviz.h>
#include <ocpl_sampling/grid_sampler.h>
#include <ocpl_sampling/halton_sampler.h>
#include <ocpl_sampling/random_sampler.h>
#include <ocpl_tsr/task_space_regions.h>

#include <ocpl_planning/planners.h>
#include <ocpl_planning/factories.h>
#include <ocpl_planning/cost_functions.h>

using namespace ocpl;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ocpl_moveit_demo");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    PlanarRobot3R robot;
    Rviz rviz;
    ros::Duration(0.2).sleep();
    rviz.clear();
    ros::Duration(0.2).sleep();

    std::vector<double> q_start = { 1.0, -0.7, 0 };
    // std::vector<double> q_start = { 0.0, 0.0, 0.0 };
    auto tf_start = robot.fk(q_start);
    auto q_start_ik = robot.ik(tf_start);

    std::cout << tf_start.translation().transpose() << std::endl;

    rviz.plotPose(tf_start);
    robot.plot(rviz.visual_tools_, q_start);

    //////////////////////////////////
    // Create task
    //////////////////////////////////
    TSRBounds bounds{ { 0.0, 0.0 }, { 0.0, 0.0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { -1, 1 } };
    std::vector<TSR> regions;
    for (int i{ 0 }; i < 10; ++i)
    {
        Transform tf = tf_start;
        tf.translation().x() = 2.6;
        tf.translation().y() = static_cast<double>(i) / 10 - 0.5;
        regions.push_back({ tf, bounds });
        rviz.plotPose(tf);
        ros::Duration(0.05).sleep();
    }

    //////////////////////////////////
    // Describe problem
    //////////////////////////////////
    auto f_is_valid = [&robot](const JointPositions& q) { return !robot.isInCollision(q); };
    // auto f_generic_inverse_kinematics = [&robot](const TSR& tsr) {
    //     static SamplerPtr sampler = createGridSampler(tsr, { 1, 1, 1, 1, 1, 10 });
    //     IKSolution result;
    //     for (auto sample : sampler->getSamples(10))
    //     {
    //         for (auto q : robot.ik(tsr.valuesToPose(sample)))
    //         {
    //             result.push_back(q);
    //         }
    //     }
    //     return result;
    // };

    // using a different sampler
    // -------------------------
    auto f_generic_inverse_kinematics = [&robot](const TSR& tsr) {
        static SamplerPtr sampler = createIncrementalSampler(tsr, SamplerType::HALTON);

        IKSolution result;
        for (auto sample : sampler->getSamples(10))
        {
            for (auto q : robot.ik(tsr.valuesToPose(sample)))
            {
                result.push_back(q);
            }
        }
        return result;
    };

    // // appart from edge costs, the graph also used node costs
    double state_weight{ 1.0 };
    auto f_state_cost = [&robot, state_weight](const TSR& tsr, const JointPositions& q) {
        double z0 = tsr.tf_nominal.rotation().eulerAngles(0, 1, 2).z();
        double z1 = robot.fk(q).rotation().eulerAngles(0, 1, 2).z();
        return state_weight * (z0 - z1) * (z0 - z1);
        // return std::abs(z1 - 0.3);
    };
    // auto f_state_cost = [](const TSR& /* tsr */, const JointPositions& /* q */) { return 0.0; };

    // //////////////////////////////////
    // // Solve problem
    // //////////////////////////////////
    // auto path = findPath(regions, L2NormDiff, f_state_cost, f_is_valid, f_generic_inverse_kinematics);

    // //////////////////////////////////
    // // Alternative solver
    // //////////////////////////////////
    std::vector<std::function<IKSolution(int)>> path_samplers;
    std::vector<Transform> nominal_tfs;
    for (auto tsr : regions)
    {
        path_samplers.push_back([tsr, &robot](int num_samples) {
            static SamplerPtr sampler = createIncrementalSampler(tsr, SamplerType::HALTON);

            IKSolution result;
            for (auto sample : sampler->getSamples(num_samples))
            {
                for (auto q : robot.ik(tsr.valuesToPose(sample)))
                {
                    result.push_back(q);
                }
            }
            return result;
        });
        nominal_tfs.push_back(tsr.tf_nominal);
    }

    // change the first path point to a specific configuration
    path_samplers[0] = [q_start_ik](int /*num_samples*/) -> IKSolution { return {q_start_ik.at(1)}; };

    auto f_state_cost_2 = [](const Transform& /* tsr */, const JointPositions& /* q */) { return 0.0; };

    auto path = findPath(path_samplers, nominal_tfs, L2NormDiff, f_state_cost_2, f_is_valid);

    for (JointPositions q : path)
    {
        if (robot.isInCollision(q))
            robot.plot(rviz.visual_tools_, q, rviz_visual_tools::RED);
        else
            robot.plot(rviz.visual_tools_, q, rviz_visual_tools::GREEN);
        ros::Duration(0.5).sleep();
    }

    return 0;
}
