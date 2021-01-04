#include <ros/ros.h>

#include <cmath>
#include <algorithm>
#include <limits>

#include <ocpl_ros/moveit_robot_examples.h>
#include <ocpl_ros/rviz.h>
#include <ocpl_sampling/grid_sampler.h>
#include <ocpl_sampling/halton_sampler.h>
#include <ocpl_sampling/random_sampler.h>
#include <ocpl_tsr/task_space_regions.h>
#include <ocpl_planning/planners.h>
#include <ocpl_planning/factories.h>

using namespace ocpl;

/** \brief Factory function to create a sampler specific to the robot of this demo.
 * 
 * Joint limits are hard coded here for now, but could be extracted from the MoveIt
 * specific robot model used in this demo. **/
SamplerPtr createGridRobotSampler(const std::vector<int>& num_samples)
{
    SamplerPtr sampler = std::make_shared<GridSampler>();
    assert(num_samples.size() == 3);
    sampler->addDimension(-1.5, 1.5, num_samples[0]);
    sampler->addDimension(-1.5, 1.5, num_samples[1]);
    sampler->addDimension(-1.5, 1.5, num_samples[2]);
    return sampler;
}

/** \brief Factory function to create a sampler specific to the robot of this demo.
 * 
 * Joint limits are hard coded here for now, but could be extracted from the MoveIt
 * specific robot model used in this demo. **/
SamplerPtr createIncrementalRobotSampler(SamplerType type)
{
    SamplerPtr sampler;
    switch (type)
    {
        case SamplerType::RANDOM: {
            sampler = std::make_shared<RandomSampler>();
            break;
        }
        case SamplerType::HALTON: {
            sampler = std::make_shared<HaltonSampler>();
            break;
        }
        default: {
            assert(false && "Jeroen, you forgot to implement a SamplerType!");
        }
    }
    sampler->addDimension(-1.5, 1.5);
    sampler->addDimension(-1.5, 1.5);
    sampler->addDimension(-1.5, 1.5);
    return sampler;
}

double L2NormDiff(NodePtr n1, NodePtr n2)
{
    double cost{};
    int s = n1->data.size();
    for (int i = 0; i < n1->data.size(); ++i)
    {
        cost += std::sqrt((n1->data[i] - n2->data[i]) * (n1->data[i] - n2->data[i]));
    }
    return cost;
}

double L1NormDiff(NodePtr n1, NodePtr n2)
{
    double cost{};
    int s = n1->data.size();
    for (int i = 0; i < n1->data.size(); ++i)
    {
        cost += std::abs(n1->data[i] - n2->data[i]);
    }
    // if (cost > 1.0)
    //     cost = std::numeric_limits<double>::max();
    return cost;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ocpl_moveit_demo_r6");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    PlanarRobot6R robot;
    Rviz rviz;
    ros::Duration(0.2).sleep();
    rviz.clear();
    ros::Duration(0.2).sleep();

    std::vector<double> q_start = { 1.0, -2.0, 1.0, -1.0, 1.0, -1.0 };
    auto tf_start = robot.fk(q_start);
    auto solution = robot.ik(tf_start, { 1.0, -1.0, 0.3 });

    std::cout << tf_start.translation().transpose() << std::endl;

    rviz.plotPose(tf_start);
    robot.plot(rviz.visual_tools_, q_start);

    ROS_INFO_STREAM("Found " << solution.size() << " ik solutions.");
    for (auto q : solution)
    {
        robot.plot(rviz.visual_tools_, q, rviz_visual_tools::GREEN);
        ros::Duration(0.5).sleep();
    }

    // for (auto q_red : sampler->getSamples())
    // {
    //     auto solution = robot.ik(tf_start, q_red);
    //     ROS_INFO_STREAM("Found " << solution.size() << " ik solutions.");
    //     for (auto q : solution)
    //     {
    //         robot.plot(rviz.visual_tools_, q, rviz_visual_tools::GREEN);
    //         ros::Duration(0.1).sleep();
    //     }
    // }

    //////////////////////////////////
    // Create task
    //////////////////////////////////
    const double small_passage_width{ 0.5 };
    Transform tf1 = Transform::Identity();
    tf1.translation() << 4.0, small_passage_width / 2, tf_start.translation().z();
    TSRBounds bounds{ { 0.0, 0.0 }, { 0.0, 0.0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, M_PI } };
    std::vector<TSR> regions;

    Transform tf = tf1;
    for (int i{ 0 }; i < 5; ++i)
    {
        tf.translation() += 0.2 * Eigen::Vector3d::UnitX();
        regions.push_back({ tf, bounds });
    }

    for (TSR& tsr : regions)
    {
        rviz.plotPose(tsr.tf_nominal);
        ros::Duration(0.05).sleep();
    }

    //////////////////////////////////
    // Describe problem
    //////////////////////////////////
    auto f_is_valid = [&robot](const JointPositions& q) { return !robot.isInCollision(q); };

    auto f_generic_inverse_kinematics = [&robot](const TSR& tsr) {
        static SamplerPtr t_sampler = createGridSampler(tsr, { 1, 1, 1, 1, 1, 30 });
        static SamplerPtr c_sampler = createGridRobotSampler({6, 6, 6});

        IKSolution result;
        for (auto t_sample : t_sampler->getSamples())
        {
            for (auto q_red : c_sampler->getSamples())
            {
                auto solution = robot.ik(tsr.valuesToPose(t_sample), q_red);
                for (auto q : solution)
                {
                    result.push_back(q);
                }
            }
        }
        return result;
    };

    // use a different sampler
    // -----------------------
    // auto f_generic_inverse_kinematics = [&robot](const TSR& tsr) {
    //     static SamplerPtr t_sampler = createIncrementalSampler(tsr, SamplerType::RANDOM);
    //     static SamplerPtr c_sampler = createIncrementalRobotSampler(SamplerType::RANDOM);

    //     IKSolution result;
    //     for (auto t_sample : t_sampler->getSamples(30))
    //     {
    //         for (auto q_red : c_sampler->getSamples(216))
    //         {
    //             auto solution = robot.ik(tsr.valuesToPose(t_sample), q_red);
    //             for (auto q : solution)
    //             {
    //                 result.push_back(q);
    //             }
    //         }
    //     }
    //     return result;
    // };

    // appart from edge costs, the graph also used node costs
    // double state_weight{ 1.0 };
    // auto f_state_cost = [&robot, state_weight](const TSR& tsr, const JointPositions& q) {
    //     double z0 = tsr.tf_nominal.rotation().eulerAngles(0, 1, 2).z();
    //     double z1 = robot.fk(q).rotation().eulerAngles(0, 1, 2).z();
    //     return state_weight * (z0 - z1) * (z0 - z1);
    //     // return std::abs(z1 - 0.3);
    // };

    auto f_state_cost = [](const TSR& /* tsr */, const JointPositions& /* q */) { return 0.0; };

    //////////////////////////////////
    // Solve problem
    //////////////////////////////////
    auto path = findPath(regions, L1NormDiff, f_state_cost, f_is_valid, f_generic_inverse_kinematics);

    for (auto q : path)
    {
        if (robot.isInCollision(q))
            robot.plot(rviz.visual_tools_, q, rviz_visual_tools::RED);
        else
            robot.plot(rviz.visual_tools_, q, rviz_visual_tools::GREEN);
        ros::Duration(0.5).sleep();
    }

    return 0;
}
