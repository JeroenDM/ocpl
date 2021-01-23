#include <ros/ros.h>

#include <algorithm>
#include <limits>
#include <chrono>

#include <ocpl_ros/moveit_robot_examples.h>
#include <ocpl_ros/rviz.h>
#include <ocpl_sampling/grid_sampler.h>
#include <ocpl_sampling/halton_sampler.h>
#include <ocpl_sampling/random_sampler.h>
#include <ocpl_tsr/task_space_regions.h>

#include <ocpl_planning/cost_functions.h>
#include <ocpl_planning/oriolo.h>

using namespace ocpl;

std::ostream& operator<<(std::ostream& os, const JointPositions& q)
{
    for (auto qi : q)
        os << qi << ", ";
    return os;
}

std::vector<TSR> createLineTask(TSRBounds bounds, Eigen::Vector3d start, Eigen::Vector3d stop,
                                Eigen::Isometry3d orientation, std::size_t num_points)
{
    std::vector<TSR> task;
    Transform tf(orientation);
    tf.translation() = start;

    Eigen::Vector3d direction = (stop - start).normalized();

    double step = (stop - start).norm() / num_points;
    for (int i{ 0 }; i < num_points; ++i)
    {
        tf.translation() += step * direction;
        task.push_back({ tf, bounds });
    }
    return task;
}

/** Case 2 from my 2018 paper. **/
std::vector<TSR> createCase2(int num_points = 5)
{
    TSRBounds bounds{ { 0.0, 0.0 }, { 0.0, 0.0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, M_PI } };
    Eigen::Vector3d start(4.0, 0.25, 0.0);
    Eigen::Vector3d stop(5.0, 0.25, 0.0);
    return createLineTask(bounds, start, stop, Eigen::Isometry3d::Identity(), num_points);
}

void showPath(const std::vector<JointPositions>& path, Rviz& rviz, std::shared_ptr<MoveItRobot> robot)
{
    for (JointPositions q : path)
    {
        if (robot->isInCollision(q))
            robot->plot(rviz.visual_tools_, q, rviz_visual_tools::RED);
        else
            robot->plot(rviz.visual_tools_, q, rviz_visual_tools::GREEN);
        ros::Duration(0.1).sleep();
    }
}

/** \brief Demo for a planar, 6 link robot.
 *
 * You can specify and load collision objects with the python script "load_collision_objects.py".
 * The main function shows how to setup and solve a task of following a simple straight line.
 *
 * */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "ocpl_moveit_demo_rn");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::shared_ptr<MoveItRobot> robot = std::make_shared<PlanarRobotNR>();
    Rviz rviz;
    ros::Duration(0.2).sleep();
    rviz.clear();
    ros::Duration(0.2).sleep();

    //////////////////////////////////
    // Create task
    //////////////////////////////////
    // small passage case
    auto regions = createCase2(30);
    // JointLimits joint_limits{ { -1.5, 1.5 }, { -1.5, 1.5 }, { -1.5, 1.5 } };  // for redundant joints
    std::vector<Bounds> joint_limits{ { -1.5, 1.5 }, { -1.5, 1.5 }, { -1.5, 1.5 },
                                      { -1.5, 1.5 }, { -1.5, 1.5 }, { -1.5, 1.5 } };  // for all joints

    std::vector<Bounds> tsr_bounds{ { 0.0, 0.0 }, { 0.0, 0.0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, M_PI } };

    // for (TSR& tsr : regions)
    // {
    //     rviz.plotPose(tsr.tf_nominal);
    //     // ros::Duration(0.05).sleep();
    // }

    auto fkFun = [&robot](const JointPositions& q) { return robot->fk(q); };

    auto ikFun = [&robot](const ocpl::Transform& tf, const JointPositions& q_red) { return robot->ik(tf, q_red); };

    auto isValidFun = [&robot](const JointPositions& q) { return !robot->isInCollision(q); };

    //////////////////////////////////
    // Try Oriolo algorithms
    //////////////////////////////////
    oriolo::Planner planner(fkFun, ikFun, isValidFun, joint_limits, tsr_bounds, robot->getNumDof() - 3);

    auto solution = planner.greedy(regions);

    if (solution.empty())
    {
        std::cout << "No solution found.\n";
    }
    {
        std::cout << "Found solution of length: " << solution.size() << "\n";
        showPath(solution, rviz, robot);
    }

    // auto solution = planner.step(0, 2, q1, regions);
    // std::cout << "step() sol size " << solution.size() << "\n";
    // //std::cout << "step() " << solution[0] << "\n";

    // solution = planner.greedy(regions);
    // std::cout << "greedy() sol size " << solution.size() << "\n";

    return 0;
}
