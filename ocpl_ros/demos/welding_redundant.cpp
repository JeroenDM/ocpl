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

#include <ocpl_planning/planners.h>
#include <ocpl_planning/factories.h>
#include <ocpl_planning/cost_functions.h>

using namespace ocpl;
using Eigen::AngleAxisd;
using Eigen::Isometry3d;
using Eigen::Vector3d;

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

std::vector<TSR> createCircleSegment(TSRBounds bounds, Transform start, Vector3d stop, Vector3d centre, Vector3d axis,
                                     std::size_t num_points)
{
    std::vector<TSR> task;

    // calculate circle segment angle
    Vector3d a = (start.translation() - centre).normalized();
    Vector3d b = (stop - centre).normalized();
    double angle = std::acos(a.dot(b));
    double step = angle / (num_points - 1);

    Isometry3d r(start.linear());
    Vector3d v = (start.translation() - centre);

    for (int i{ 0 }; i < num_points; ++i)
    {
        Transform tf(r);
        tf.translation() = centre + v;
        task.push_back({ tf, bounds });
        v = AngleAxisd(step, axis) * v;
        r = AngleAxisd(step, axis) * r;
    }
    return task;
}

void showPath(const std::vector<JointPositions>& path, Rviz& rviz, MoveItRobot& robot)
{
    for (JointPositions q : path)
    {
        if (robot.isInCollision(q))
            robot.plot(rviz.visual_tools_, q, rviz_visual_tools::RED);
        else
            robot.plot(rviz.visual_tools_, q, rviz_visual_tools::DEFAULT);
        ros::Duration(0.1).sleep();
    }
}

inline double deg2rad(double deg)
{
    return deg * M_PI / 180.0;
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

    IndustrialRobot robot("tool_tip");
    Rviz rviz;
    ros::Duration(0.2).sleep();
    rviz.clear();
    ros::Duration(0.2).sleep();

    // create an interesting start configurations
    std::vector<double> q_start(robot.getNumDof(), 1.0);

    auto tf_start = robot.fk(q_start);
    // std::cout << tf_start.translation().transpose() << std::endl;

    // rviz.plotPose(tf_start);
    // robot.plot(rviz.visual_tools_, q_start, rviz_visual_tools::MAGENTA);
    // ros::Duration(0.2).sleep();

    // // for (auto q_red : sampler->getSamples())
    // // {
    // auto solution = robot.ik(tf_start, {0.5});
    // ROS_INFO_STREAM("Found " << solution.size() << " ik solutions.");
    // for (auto q : solution)
    // {
    //     robot.plot(rviz.visual_tools_, q, rviz_visual_tools::GREEN);
    //     ros::Duration(0.2).sleep();
    // }
    // // }

    //////////////////////////////////
    // Create task
    //////////////////////////////////
    double da = deg2rad(10);
    TSRBounds bounds{ { 0, 0 }, { 0, 0 }, { 0, 0 }, { -da, da }, { -da, da }, { -M_PI, M_PI } };
    // TSRBounds bounds{ { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { -M_PI, M_PI } };
    Vector3d work(2, 0.3, 0.9027);
    Vector3d start(0.0, 1.05, 0.05);
    Vector3d stop(0.9, 1.05, 0.05);
    Isometry3d ori(AngleAxisd(deg2rad(135), Vector3d::UnitX()) * AngleAxisd(0.0, Vector3d::UnitY()) *
                   AngleAxisd(0.0, Vector3d::UnitZ()));
    // auto regions = createLineTask(bounds, start + work, stop + work, ori, 20);

    // add another line
    Vector3d start_2(0.95, 1.1, 0.05);
    Vector3d stop_2(0.95, 2.9, 0.05);
    Isometry3d ori_2(AngleAxisd(0.0, Vector3d::UnitX()) * AngleAxisd(deg2rad(135), Vector3d::UnitY()) *
                     AngleAxisd(deg2rad(90), Vector3d::UnitZ()));
    auto second_line = createLineTask(bounds, start_2 + work, stop_2 + work, ori_2, 40);
    auto regions = second_line;

    // // and add another line
    // Vector3d start_3(0.9, 2.95, 0.05);
    // Vector3d stop_3(0.0, 2.95, 0.05);
    // Isometry3d ori_3(AngleAxisd(deg2rad(-135), Vector3d::UnitX()) * AngleAxisd(0.0, Vector3d::UnitY()) *
    //                  AngleAxisd(deg2rad(180), Vector3d::UnitZ()));
    // auto third_line = createLineTask(bounds, start_3 + work, stop_3 + work, ori_3, 20);

    // // connect them with a circle segment
    // Transform tf_a(ori);
    // tf_a.translation() = stop + work;
    // Vector3d centre(0.9, 1.1, 0.05);
    // centre += work;
    // auto circle_segment = createCircleSegment(bounds, tf_a, start_2 + work, centre, Eigen::Vector3d::UnitZ(), 10);

    // Transform tf_a_2(ori_2);
    // tf_a_2.translation() = stop_2 + work;
    // Vector3d centre_2(0.9, 2.9, 0.05);
    // centre_2 += work;
    // auto circle_segment_2 = createCircleSegment(bounds, tf_a_2, start_3 + work, centre_2, Eigen::Vector3d::UnitZ(), 10);

    // regions.insert(regions.end(), circle_segment.begin(), circle_segment.end());
    // regions.insert(regions.end(), second_line.begin(), second_line.end());
    // regions.insert(regions.end(), circle_segment_2.begin(), circle_segment_2.end());
    // regions.insert(regions.end(), third_line.begin(), third_line.end());

    JointLimits joint_limits{ { 0.0, 3.0 } };

    EigenSTL::vector_Vector3d visual_path;
    for (TSR& tsr : regions)
    {
        rviz.plotPose(tsr.tf_nominal);
        ros::Duration(0.05).sleep();
        visual_path.push_back(tsr.tf_nominal.translation());
    }
    // rviz.visual_tools_->publishPath(visual_path);
    // rviz.visual_tools_->trigger();

    //////////////////////////////////
    // Simple interface solver
    //////////////////////////////////
    auto redundant_joint_limits = joint_limits;

    // function that tells you whether a state is valid (collision free)
    auto is_valid_fun = [&robot](const JointPositions& q) { return !robot.isInCollision(q); };

    // function that returns analytical inverse kinematics solution for end-effector pose
    auto ik_fun = [&robot](const Transform& tf, const JointPositions& q_fixed) { return robot.ik(tf, q_fixed); };

    // specify an objective to minimize the cost along the path
    // here we use a predefined function that uses the L1 norm of the different
    // between two joint positions
    // auto path_cost_fun = L1NormDiff2;

    // // try to limit joint speed
    std::vector<double> max_joint_speed(robot.getNumDof(), 0.5);

    auto path_cost_fun = [&max_joint_speed](const std::vector<double>& n1, const std::vector<double>& n2) {
        assert(n1.size() == n2.size());
        assert(max_joint_speed.size() == n1.size());

        double cost{ 0.0 };
        for (int i = 0; i < n1.size(); ++i)
        {
            double inc = std::abs(n1[i] - n2[i]);
            inc = std::min(inc, 2 * M_PI - inc);
            if (inc > max_joint_speed[i])
                return std::nan("1");
            cost += inc;
        }
        // if (cost > 1.0)
        //     cost = std::numeric_limits<double>::max();
        return cost;
    };

    // optionally we can set a state cost for every point along the path
    // this one tries to keep the end-effector pose close the the nominal pose
    // defined in the task space region
    auto state_cost_fun = [&robot](const TSR& tsr, const JointPositions& q) {
        return poseDistance(tsr.tf_nominal, robot.fk(q)).norm();
    };
    // auto state_cost_fun = zeroStateCost;

    // settings to select a planner
    PlannerSettings ps;
    ps.is_redundant = true;
    // ps.sampler_type = SamplerType::HALTON;
    // ps.t_space_batch_size = 20;
    // ps.c_space_batch_size = 10;
    // ps.min_valid_samples = 3000;
    // ps.max_iters = 100000;

    // ps.sampler_type = SamplerType::GRID;
    // ps.tsr_resolution = { 1, 1, 1, 1, 1, 100 };
    // ps.redundant_joints_resolution = std::vector<int>{ 100 };
    ps.tsr_resolution = { 1, 1, 1, 1, 1, 60 };
    ps.redundant_joints_resolution = std::vector<int>{ 30 };

    // solve it!
    Solution res = solve(regions, joint_limits, ik_fun, is_valid_fun, path_cost_fun, state_cost_fun, ps);
    if (res.success)
    {
        std::cout << "A solution is found with a cost of " << res.cost << "\n";
    }
    else
    {
        std::cout << "No complete solution was found.\n";
    }

    showPath(res.path, rviz, robot);

    return 0;
}
