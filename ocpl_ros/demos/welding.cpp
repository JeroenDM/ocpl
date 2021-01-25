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
#include <ocpl_planning/settings.h>

// ROS Trajectory Action server definition
#include <control_msgs/FollowJointTrajectoryAction.h>
// Means by which we communicate with above action-server
#include <actionlib/client/simple_action_client.h>

using namespace ocpl;

void showPath(const std::vector<JointPositions>& path, Rviz& rviz, MoveItRobot& robot, double dt = 0.5)
{
    for (JointPositions q : path)
    {
        if (robot.isInCollision(q))
            robot.plot(rviz.visual_tools_, q, rviz_visual_tools::RED);
        else
            // robot.plot(rviz.visual_tools_, q, rviz_visual_tools::GREEN);
            robot.plot(rviz.visual_tools_, q, rviz_visual_tools::DEFAULT);
        ros::Duration(dt).sleep();
    }
}

/** \brief Read the data from the puzzle piece from the Descartes tutorials.
 * code copied from:
 * https://github.com/ros-industrial-consortium/descartes_tutorials/blob/master/descartes_tutorials/src/tutorial2.cpp
 *
 * (And the corresponding csv file is also copied from these tutorials.)
 * **/
static EigenSTL::vector_Isometry3d makePuzzleToolPoses()
{
    EigenSTL::vector_Isometry3d path;  // results
    std::ifstream indata;              // input file

    // You could load your parts from anywhere, but we are transporting them with the git repo
    std::string filename = ros::package::getPath("ocpl_ros") + "/data/puzzle_descartes.csv";

    // In a non-trivial app, you'll of course want to check that calls like 'open' succeeded
    indata.open(filename);

    std::string line;
    int lnum = 0;
    while (std::getline(indata, line))
    {
        ++lnum;
        if (lnum < 3)
            continue;

        std::stringstream lineStream(line);
        std::string cell;
        Eigen::Matrix<double, 6, 1> xyzijk;
        int i = -2;
        while (std::getline(lineStream, cell, ','))
        {
            ++i;
            if (i == -1)
                continue;

            xyzijk(i) = std::stod(cell);
        }

        Eigen::Vector3d pos = xyzijk.head<3>();
        pos = pos / 1000.0;  // Most things in ROS use meters as the unit of length. Our part was exported in mm.
        Eigen::Vector3d norm = xyzijk.tail<3>();
        norm.normalize();

        // This code computes two extra directions to turn the normal direction into a full defined frame. Descartes
        // will search around this frame for extra poses, so the exact values do not matter as long they are valid.
        Eigen::Vector3d temp_x = (-1 * pos).normalized();
        Eigen::Vector3d y_axis = (norm.cross(temp_x)).normalized();
        Eigen::Vector3d x_axis = (y_axis.cross(norm)).normalized();
        Eigen::Isometry3d pose;
        pose.matrix().col(0).head<3>() = x_axis;
        pose.matrix().col(1).head<3>() = y_axis;
        pose.matrix().col(2).head<3>() = norm;
        pose.matrix().col(3).head<3>() = pos;

        path.push_back(pose);
    }
    indata.close();

    return path;
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

inline double deg2rad(double deg)
{
    return deg * M_PI / 180.0;
}

int main(int argc, char** argv)
{
    using Eigen::AngleAxisd;
    using Eigen::Vector3d;

    ros::init(argc, argv, "ocpl_moveit_demo");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    IndustrialRobot robot("tool_tip");
    // IndustrialRobot robot("tool0");
    Rviz rviz("base_link");
    rviz.clear();

    std::vector<double> q_start = { 1.0, -1.0, 1.0, -1.0, 1.0, -1.0 };
    std::vector<double> q_zero(robot.getNumDof(), 0.0);
    // std::vector<double> q_start = { 0.0, 0.0, 0.0 };
    auto tf_start = robot.fk(q_start);
    auto q_start_ik = robot.ik(tf_start);

    // std::cout << tf_start.translation().transpose() << std::endl;

    // rviz.plotPose(tf_start);
    // robot.plot(rviz.visual_tools_, q_start);
    // ros::Duration(0.5).sleep();

    // std::cout << "Found " << q_start_ik.size() << " ik solutions.\n";
    // for (auto q : q_start_ik)
    // {
    //     robot.plot(rviz.visual_tools_, q);
    //     ros::Duration(0.5).sleep();
    // }

    //////////////////////////////////
    // l_profile
    //////////////////////////////////
    // TSRBounds bounds{ { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { -M_PI, M_PI } };
    // Transform tf = robot.fk(q_zero);

    // Eigen::Isometry3d ori(AngleAxisd(0.0, Vector3d::UnitX()) * AngleAxisd(deg2rad(135), Vector3d::UnitY()) *
    //                       AngleAxisd(deg2rad(90), Vector3d::UnitZ()));
    // Eigen::Vector3d start(0.98, -0.5, 0.02);
    // Eigen::Vector3d stop(0.98, 0.5, 0.02);
    // const std::size_t num_points{ 30 };
    // const double total_distance = (stop - start).norm();
    // std::vector<TSR> task = createLineTask(bounds, start, stop, ori, num_points);

    // EigenSTL::vector_Vector3d visual_path;
    // for (auto tsr : task)
    // {
    //     rviz.plotPose(tsr.tf_nominal);
    //     visual_path.push_back(tsr.tf_nominal.translation());
    //     ros::Duration(0.1).sleep();
    // }
    // ros::Duration(0.5).sleep();

    //////////////////////////////////
    // glass of water
    //////////////////////////////////
    TSRBounds bounds{ { 0.0, 0.0 }, { 0, 0 }, { -0.05, 0.05 }, { -1.3, 1.3 }, { 0, 0 }, { 0.0, 0.0 } };

    Eigen::Isometry3d ori(AngleAxisd(deg2rad(90), Vector3d::UnitY()) * AngleAxisd(deg2rad(-90), Vector3d::UnitX()));
    Eigen::Vector3d start(0.98, 0.0, 0.8);

    Transform tf(ori);
    tf.translation() = start;

    const std::size_t num_points{ 30 };
    const double total_distance = 1.1;
    double step = total_distance / num_points;

    std::vector<TSR> task;
    for (int i{ 0 }; i < num_points; ++i)
    {
        tf = tf * Eigen::AngleAxisd(step, Eigen::Vector3d::UnitY());
        task.push_back(TSR{ tf, bounds });
        task.back().local_ = false;
    }

    EigenSTL::vector_Vector3d visual_path;
    int skipper{ 0 };
    for (auto tsr : task)
    {
        if (skipper % 3 == 0)
        {
            rviz.plotPose(tsr.tf_nominal);
            visual_path.push_back(tsr.tf_nominal.translation());
            ros::Duration(0.1).sleep();
        }
        skipper++;
    }
    ros::Duration(0.5).sleep();

    //////////////////////////////////
    // Puzzle
    //////////////////////////////////
    // TSRBounds bounds{ { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { -M_PI, M_PI } };
    // Transform tf = robot.fk(q_zero);
    // auto task_tfs = makePuzzleToolPoses();
    // std::size_t num_points = task_tfs.size();
    // EigenSTL::vector_Vector3d visual_path;
    // std::vector<TSR> task;
    // for (auto& pose : task_tfs)
    // {
    //     pose.translation().x() += 0.8;
    //     task.push_back({ pose, bounds });
    //     visual_path.push_back(pose.translation());
    // }
    // double total_distance{0.0};
    // for (std::size_t i{1}; i < task_tfs.size(); ++i)
    // {
    //     total_distance += (task_tfs[i].translation() - task_tfs[i-1].translation()).norm();
    // }

    // rviz.visual_tools_->publishPath(visual_path);
    // ros::Duration(1.0).sleep();

    //////////////////////////////////
    // Simple interface solver
    //////////////////////////////////
    // joint limits for the redundant joints
    JointLimits joint_limits{};

    // function that tells you whether a state is valid (collision free)
    auto f_is_valid = [&robot](const JointPositions& q) { return !robot.isInCollision(q); };
    // auto f_is_valid = [&robot](const JointPositions& q) { return true; };

    // function that returns analytical inverse kinematics solution for end-effector pose
    auto f_ik = [&robot](const Transform& tf, const JointPositions& /*q_fixed*/) { return robot.ik(tf); };

    // calculate max step size for joints based on max speed
    // TODO get this from robot model!
    std::vector<double> max_joint_speed{ 154, 154, 228, 343, 384, 721 };  // radians / second
    // convert to radians and scale:
    for (double& s : max_joint_speed)
    {
        s = 1.0 * deg2rad(s);
    }
    // welding speed 10 cm / seconde? 0.1 m/s
    const double welding_speed{ 0.1 };
    const double dt = (total_distance / welding_speed) / num_points;

    std::cout << "distance: " << total_distance << "\n";
    std::cout << "dt: " << dt << "\n";
    std::cout << "max dq: " << dt * max_joint_speed[0] << "\n";

    // specify an objective to minimize the cost along the path
    // here we use a predefined function that uses the L1 norm of the different
    // between two joint positions
    auto f_path_cost = L2NormDiff2;
    // auto f_path_cost = [&max_joint_speed, dt](const std::vector<double>& n1, const std::vector<double>& n2) {
    //     assert(n1.size() == n2.size());
    //     assert(max_joint_speed.size() == n1.size());

    //     double cost{ 0.0 };
    //     for (int i = 0; i < n1.size(); ++i)
    //     {
    //         double inc = std::abs(n1[i] - n2[i]);
    //         if (inc > max_joint_speed[i] * dt)
    //             return std::nan("1");
    //         cost += inc;
    //     }
    //     // if (cost > 1.0)
    //     //     cost = std::numeric_limits<double>::max();
    //     return cost;
    // };

    // optionally we can set a state cost for every point along the path
    // this one tries to keep the end-effector pose close the the nominal pose
    // defined in the task space region
    auto f_state_cost = [&robot](const TSR& tsr, const JointPositions& q) {
        return 1.0 * poseDistance(tsr.tf_nominal, robot.fk(q)).norm();
    };
    // auto f_state_cost = zeroStateCost;

    // keep close to robot home pose
    std::vector<double> q_home{ 0, -1.5708, 1.5708, 0, 0, 0 };
    // robot.plot(rviz.visual_tools_, q_home, rviz_visual_tools::MAGENTA);
    // ros::Duration(0.5).sleep();

    // auto f_state_cost = [q_home](const TSR& tsr, const JointPositions& q) { return L2NormDiff2(q, q_home); };

    // settings to select a planner
    PlannerSettings ps;
    ps.sampler_type = SamplerType::HALTON;
    ps.t_space_batch_size = 100;
    // ps.c_space_batch_size = 100;
    ps.min_valid_samples = 100;
    ps.max_iters = 500;

    // ps.sampler_type = SamplerType::GRID;
    // ps.tsr_resolution = { 1, 1, 1, 1, 1, 10 };

    // // solve it!
    auto solution = solve(task, joint_limits, f_ik, f_is_valid, f_path_cost, f_state_cost, ps);

    showPath(solution.path, rviz, robot, dt);

    return 0;
}
