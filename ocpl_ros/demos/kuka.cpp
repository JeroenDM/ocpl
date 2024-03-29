#include <ros/ros.h>

#include <algorithm>

#include <simple_moveit_wrapper/industrial_robot.h>

#include <ocpl_ros/rviz.h>
#include <ocpl_ros/io.h>

#include <ocpl_sampling/grid_sampler.h>
#include <ocpl_sampling/halton_sampler.h>
#include <ocpl_sampling/random_sampler.h>
#include <ocpl_tsr/task_space_regions.h>

#include <ocpl_planning/acro_planner.h>
#include <ocpl_planning/factories.h>
#include <ocpl_planning/cost_functions.h>
#include <ocpl_planning/types.h>

#include <ocpl_ros/planning_cases/puzzle.h>
#include <ocpl_ros/planning_cases/teapot.h>
#include <ocpl_ros/planning_cases/l_profile.h>

#include <ocpl_benchmark/benchmark.h>

using namespace ocpl;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ocpl_moveit_demo");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    simple_moveit_wrapper::IndustrialRobot robot("manipulator", "tool_tip");
    // IndustrialRobot robot("tool0");
    Rviz rviz("base_link");
    rviz.clear();

    //////////////////////////////////
    // Planning settings
    //////////////////////////////////
    // PlannerSettings ps;
    // ps.is_redundant = true;
    // ps.sampler_type = SamplerType::HALTON;
    // // ps.sampler_type = SamplerType::RANDOM;
    // ps.t_space_batch_size = 10;
    // ps.c_space_batch_size = 100;
    // ps.min_valid_samples = 100;
    // ps.max_iters = 500;

    // default settings
    std::string PLANNER_SETTINGS_FILE{ "kuka/kuka_gpq.yaml" };
    if (argc > 2)
    {
        PLANNER_SETTINGS_FILE = std::string(argv[2]);
    }
    ROS_INFO_STREAM("Using planner settings file: " << PLANNER_SETTINGS_FILE);
    PlannerSettings ps = loadSettingsFromFile(PLANNER_SETTINGS_FILE);

    //////////////////////////////////
    // Select the planning task
    // And get the case specific parameters
    //////////////////////////////////
    // some default settings
    std::function<double(const std::vector<double>& n1, const std::vector<double>& n2)> path_cost_fun = L2NormDiff2;
    std::function<double(const TSR& tsr, const JointPositions& q)> state_cost_fun = zeroStateCost;

    int PLANNING_CASE{ 1 };
    if (argc > 1)
    {
        PLANNING_CASE = std::stoi(argv[1]);
    }
    ROS_INFO_STREAM("Selected planning case " << PLANNING_CASE);

    std::vector<TSR> task;
    std::vector<std::vector<TSR>> multiple_tasks;
    switch (PLANNING_CASE)
    {
        case 1: {
            //////////////////////////////////
            // l_profile
            //////////////////////////////////
            task = l_profile::waypoints(/* x_offset */ 0.98, /* extra_tolerance */ true);
            for (auto tsr : task)
            {
                rviz.plotPose(tsr.tf_nominal);
                ros::Duration(0.05).sleep();
            }
            ros::Duration(0.1).sleep();
            // optionally we can set a state cost for every point along the path
            // this one tries to keep the end-effector pose close the the nominal pose
            // defined in the task space region
            state_cost_fun = [&robot, &ps](const TSR& tsr, const JointPositions& q) {
                Eigen::VectorXd dv = poseDistance(tsr.tf_nominal, robot.fk(q));
                // only penalize the x and y rotation
                return ps.state_cost_weight * std::sqrt(dv[3] * dv[3] + dv[4] * dv[4]);
            };
            break;
        }
        case 2: {
            //////////////////////////////////
            // glass of water
            //////////////////////////////////
            task = teapot::waypoints();

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
            break;
        }

        case 3: {
            //////////////////////////////////
            // Puzzle
            //////////////////////////////////
            auto welding_task = puzzle::waypoints();

            // select only the first 100 points
            task.reserve(100);
            std::copy_n(welding_task.begin(), 100, std::back_inserter(task));

            EigenSTL::vector_Vector3d path_positions;
            for (auto& pt : task)
            {
                path_positions.push_back(pt.tf_nominal.translation());
            }

            rviz.visual_tools_->publishPath(path_positions);
            ros::Duration(1.0).sleep();
            break;
        }
        case 4: {
            //////////////////////////////////
            // from generic csv file
            //////////////////////////////////

            multiple_tasks = ocpl::readPathsFromCsvFile("kingpin2.csv");
            task = multiple_tasks.at(2);

            ROS_INFO_STREAM("task length: " << task.size());

            EigenSTL::vector_Vector3d path_positions;
            for (auto& t : multiple_tasks)
            {
                path_positions.clear();
                for (auto& pt : t)
                {
                    path_positions.push_back(pt.tf_nominal.translation());
                }
                rviz.visual_tools_->publishPath(path_positions);
                rviz.visual_tools_->trigger();
                ros::Duration(0.1).sleep();
            }

            break;
        }

        default: {
            ROS_WARN("Unkown planning case id.");
            return 0;
        }
    }
    std::vector<Bounds> tsr_bounds = task.at(0).bounds.asVector();  // use first waypoint bounds for all waypoints

    //////////////////////////////////
    // Translate robot kinematics to solver interface
    //////////////////////////////////
    // function that tells you whether a state is valid (collision free)
    auto f_is_valid = [&robot](const JointPositions& q) { return !robot.isColliding(q); };
    // auto f_is_valid = [&robot](const JointPositions& q) { return true; };

    // function that returns analytical inverse kinematics solution for end-effector pose
    auto f_ik = [&robot](const Transform& tf, const JointPositions& /*q_fixed*/) { return robot.ik(tf); };

    // covert simple_moveit_wrapper limits to ocpl limits
    // (they are the exactly the same struct, but I'm not sure how to share the common data type)
    JointLimits joint_limits;
    for (auto bound : robot.getJointPositionLimits())
    {
        joint_limits.push_back(ocpl::Bounds{ bound.lower, bound.upper });
    }

    Robot bot{ robot.getNumDof(),
               robot.getNumRedDof(),
               joint_limits,
               [&robot](const JointPositions& q) { return robot.fk(q); },
               f_ik,
               f_is_valid };

    // std::vector<double> q_start = { 1.0, -1.0, 1.0, -1.0, 1.0, -1.0 };
    // std::vector<double> q_zero(robot.getNumDof(), 0.0);
    // // std::vector<double> q_start = { 0.0, 0.0, 0.0 };
    // auto tf_start = robot.fk(q_start);
    // auto q_start_ik = robot.ik(tf_start);

    // // define a path cost that puts an upper limit on the joint motion between two waypoints
    // // this limit is calculated from the desired end-effector speed along the path and the robot's joint velocity
    // limits
    // auto dq_max = calcMaxJointStep(task, robot.getJointVelocityLimits(), /* end-effector speed */ 0.05);
    // auto f_path_cost_fun = [&dq_max](const std::vector<double>& n1, const std::vector<double>& n2) {
    //     assert(n1.size() == n2.size());
    //     assert(dq_max.size() == n1.size());

    //     double cost{ 0.0 };
    //     for (int i = 0; i < n1.size(); ++i)
    //     {
    //         double inc = std::abs(n1[i] - n2[i]);
    //         if (inc > dq_max[i])
    //             return std::nan("1");
    //         cost += inc;
    //     }
    //     // if (cost > 1.0)
    //     //     cost = std::numeric_limits<double>::max();
    //     return cost;
    // };

    // auto f_state_cost_fun = zeroStateCost;

    // ps.sampler_type = SamplerType::GRID;
    // ps.tsr_resolution = { 1, 1, 1, 1, 1, 10 };

    // welding specific state cost function
    // state_cost_fun = [&robot, &ps](const TSR& tsr, const JointPositions& q) {
    //     Eigen::VectorXd dv = poseDistance(tsr.tf_nominal, robot.fk(q));
    //     // only penalize the x and y rotation
    //     return ps.state_cost_weight * std::sqrt(dv[3] * dv[3] + dv[4] * dv[4]);
    // };

    // state_cost_fun = [&robot](const TSR& tsr, const JointPositions& q) {
    //     return poseDistance(tsr.tf_nominal, robot.fk(q)).norm();
    // };

    // keep close to robot home pose
    std::vector<double> q_home{ 0, -1.5708, 1.5708, 0, 0, 0 };
    // robot.plot(rviz.visual_tools_, q_home, rviz_visual_tools::MAGENTA);
    // ros::Duration(0.5).sleep();

    state_cost_fun = [q_home](const TSR& tsr, const JointPositions& q) { return L2NormDiff2(q, q_home); };

    //////////////////////////////////
    // Solve it
    //////////////////////////////////
    UnifiedPlanner planner(bot, ps);

    if (!multiple_tasks.empty())
    {
        for (auto current_task : multiple_tasks)
        {
            // // solve it!
            auto solution = planner.solve(current_task, path_cost_fun, state_cost_fun);

            robot.animatePath(rviz.visual_tools_, solution.path);
        }
    }
    else
    {
        // // solve it!
        auto solution = planner.solve(task, path_cost_fun, state_cost_fun);

        robot.animatePath(rviz.visual_tools_, solution.path);
    }

    // //////////////////////////////////
    // // Benchmark
    // //////////////////////////////////
    // // std::vector<std::string> file_names = readLinesFromFile("kuka/filename_list.txt");
    // // std::vector<PlannerSettings> settings;
    // // ROS_INFO("Running benchmark for the settings files:");
    // // for (auto name : file_names)
    // // {
    // //     ROS_INFO_STREAM(name);
    // //     settings.push_back(loadSettingsFromFile(name));

    // //     // change case and robot specific settings
    // //     // settings.back().tsr_resolution = case_settings.tsr_resolution;
    // //     // settings.back().redundant_joints_resolution = case_settings.redundant_joints_resolution;
    // // }

    // // read the base settings for the three sample methods from files
    // auto grid = loadSettingsFromFile("kuka/grid.yaml");
    // auto halton = loadSettingsFromFile("kuka/halton.yaml");
    // auto halton_i = loadSettingsFromFile("kuka/halton_incremental.yaml");
    // auto random = loadSettingsFromFile("kuka/random.yaml");
    // auto random_i = loadSettingsFromFile("kuka/random_incremental.yaml");

    // // no create variations with differt grid resolutions
    // std::vector<PlannerSettings> settings;
    // // std::vector<std::array<int, 2>> grid_res{ { 3, 10 }, { 5, 30 }, { 10, 60 }, {10, 100} };
    // // for (auto res : grid_res)
    // // {
    // //     PlannerSettings new_settings = grid;
    // //     new_settings.name = grid.name + "_" + std::to_string(res[0] * res[1]);
    // //     new_settings.tsr_resolution = { 1, 1, 1, 1, res[0], res[1] };
    // //     settings.push_back(new_settings);
    // // }
    // std::vector<int> batch_sizes {30, 150, 600, 1000};
    // for (auto bs : batch_sizes)
    // {
    //     PlannerSettings new_h = halton;
    //     new_h.name = halton.name + "_" + std::to_string(bs);
    //     new_h.t_space_batch_size = bs;
    //     settings.push_back(new_h);

    //     PlannerSettings new_h_i = halton_i;
    //     new_h_i.name = halton_i.name + "_" + std::to_string(bs);
    //     new_h_i.min_valid_samples = bs;
    //     new_h_i.max_iters = 10 * bs;
    //     settings.push_back(new_h_i);

    //     PlannerSettings new_r = random;
    //     new_r.name = random.name + "_" + std::to_string(bs);
    //     new_r.t_space_batch_size = bs;
    //     settings.push_back(new_r);

    //     PlannerSettings new_r_i = random_i;
    //     new_r_i.name = random_i.name + "_" + std::to_string(bs);
    //     new_r_i.min_valid_samples = bs;
    //     new_r_i.max_iters = 10 * bs;
    //     settings.push_back(new_r_i);
    // }

    // UnifiedPlanner planner(bot, halton);
    // auto solution = planner.solve(task);
    // robot.animatePath(rviz.visual_tools_, solution.path);

    // std::string outfilename{ "results/teapot" + std::to_string(PLANNING_CASE) + ".csv" };
    // runBenchmark(outfilename, bot, task, planner, settings, 5);

    return 0;
}
