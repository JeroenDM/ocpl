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
        ros::Duration(0.5).sleep();
    }
}

namespace oriolo
{
namespace magic
{
// d: "maximum allowed displacement of a single joint."
constexpr double D{ 0.1 };
// upperbound on calls to randConf to find joint positions for a waypoint along thepath
constexpr size_t MAX_SHOTS{ 100 };
// maximum iterations to find a good start configuration for greedy search
constexpr size_t MAX_ITER{ 1000 };
}  // namespace magic

SamplerPtr createRedundantSampler(const size_t num_red_joints)
{
    auto sampler = std::make_shared<RandomSampler>();
    for (size_t i{ 0 }; i < num_red_joints; ++i)
    {
        sampler->addDimension(-magic::D, magic::D);
    }
    return sampler;
}

/** \brief Simple interpolation between two vectors. Returns one vector.**/
template <typename T>
inline std::vector<T> interpolate(std::vector<T> q_from, std::vector<T> q_to, T s)
{
    std::vector<T> q(q_from.size());
    for (std::size_t i = 0; i < q_from.size(); ++i)
    {
        q[i] = (1 - s) * q_from[i] + s * q_to[i];
    }
    return q;
}

class Oriolo
{
  private:
    SamplerPtr red_sampler_;
    SamplerPtr q_sampler_;
    SamplerPtr tsr_sampler_;
    std::shared_ptr<MoveItRobot> robot_;
    size_t NUM_RED_DOF_{ 0 };

  public:
    Oriolo(std::shared_ptr<MoveItRobot> robot, const JointLimits& joint_limits, const std::vector<Bounds>& tsr_bounds)
      : robot_(robot)
    {
        // sampler to generate perturbations on redundant joints with max deviation d
        red_sampler_ = oriolo::createRedundantSampler(3);
        // sampler to generate completely random robot configurations inside the limits
        q_sampler_ = createIncrementalSampler(joint_limits, SamplerType::RANDOM);
        // sampler to generate random valid end-effector poses paramterised with 6 element vectors
        tsr_sampler_ = createIncrementalSampler(tsr_bounds, SamplerType::RANDOM);

        NUM_RED_DOF_ = joint_limits.size() - 3;
    }
    /** \brief Inverse kinematics with a bias value to stay close to.
     *
     * Returns empty vector if it failed.
     * **/
    JointPositions invKin(const TSR& tsr, const JointPositions& q_red, const JointPositions& q_bias)
    {
        Transform tf = tsr.valuesToPose(tsr_sampler_->getSamples(1)[0]);
        IKSolution sol = robot_->ik(tf, q_red);
        for (auto q_sol : sol)
        {
            if (LInfNormDiff2(q_sol, q_bias) < magic::D)
            {
                return q_sol;
            }
        }
        return {};
    }

    JointPositions invKin(const TSR& tsr, const JointPositions& q_red)
    {
        Transform tf = tsr.valuesToPose(tsr_sampler_->getSamples(1)[0]);
        IKSolution sol = robot_->ik(tf, q_red);
        if (sol.empty())
        {
            return {};
        }
        else
        {
            return sol.back();
        }
    }

    /** \brief return random positions for redundant joints. Centered around a bias position.
     *
     * In paper it is described as "generated through a limited random perturbation of q_red_bias".
     * I interpret limited as using the maximum displacement parameter d;
     * **/
    JointPositions randRed(const JointPositions& q_bias)
    {
        JointPositions q_red_random(NUM_RED_DOF_);
        JointPositions perturbation = red_sampler_->getSamples(1)[0];
        for (size_t i{ 0 }; i < NUM_RED_DOF_; ++i)
            q_red_random[i] = q_bias[i] + perturbation[i];
        return q_red_random;
    };

    JointPositions randConf()
    {
        return q_sampler_->getSamples(1)[0];
    }

    JointPositions randConf(const TSR& tsr)
    {
        auto q_random = randConf();
        return invKin(tsr, q_random);  // this is an emtpy vector if invKin failed
    }

    JointPositions randConf(const TSR& tsr, const JointPositions& q_bias)
    {
        auto q_red = randRed(q_bias);
        auto q_ik = invKin(tsr, q_red, q_bias);
        return q_ik;  // this is an emtpy vector if invKin failed
    }

    bool noColl(const JointPositions& q)
    {
        return !robot_->isInCollision(q);
    }

    bool noColl(const JointPositions& q_from, const JointPositions& q_to)
    {
        if (robot_->isInCollision(q_from))
            return false;
        // TODO figure out what they do mean in the paper
        // now fix the number of steps
        const int steps{ 3 };
        for (int step = 1; step < steps; ++step)
        {
            auto q_step = interpolate(q_from, q_to, static_cast<double>(step) / (steps - 1));
            if (robot_->isInCollision(q_step))
            {
                return false;
            }
        }
        return true;
    }

    std::vector<JointPositions> step(size_t start_index, size_t stop_index, const JointPositions& q_start,
                                     const std::vector<TSR>& task)
    {
        assert(start_index >= 0);
        assert(stop_index < task.size());

        JointPositions q_current = q_start;
        JointPositions q_new(q_start.size());
        std::vector<JointPositions> path;
        path.push_back(q_start);

        for (size_t j = start_index; j < stop_index; ++j)
        {
            size_t l{ 0 };
            bool success{ false };
            while (l < magic::MAX_SHOTS && !success)
            {
                JointPositions q_new = randConf(task[j + 1], q_current);
                if (!q_new.empty() && noColl(q_current, q_new))
                {
                    success = true;
                }
                l++;
            }
            if (l == magic::MAX_SHOTS)
            {
                return {};
            }
            else
            {
                path.push_back(q_new);
                q_current = q_new;
            }
        }
        return path;
    }

    std::vector<JointPositions> greedy(const std::vector<TSR>& task)
    {
        std::vector<JointPositions> path;
        size_t iters{};
        bool success{ false };
        while (iters < magic::MAX_ITER && !success)
        {
            JointPositions q_start = randConf(task[0]);
            path = step(0, task.size() - 1, q_start, task);
            iters++;
            if (!path.empty())
            {
                success = true;
            }
        }
        if (success)
            return path;
        else
            return {};
    }
};

}  // namespace oriolo

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
    auto regions = createCase2(20);
    // JointLimits joint_limits{ { -1.5, 1.5 }, { -1.5, 1.5 }, { -1.5, 1.5 } };  // for redundant joints
    JointLimits joint_limits{
        { -1.5, 1.5 }, { -1.5, 1.5 }, { -1.5, 1.5 }, { -1.5, 1.5 }, { -1.5, 1.5 }, { -1.5, 1.5 }
    };  // for all joints

    JointLimits tsr_bounds{ { 0.0, 0.0 }, { 0.0, 0.0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, M_PI } };

    // for (TSR& tsr : regions)
    // {
    //     rviz.plotPose(tsr.tf_nominal);
    //     // ros::Duration(0.05).sleep();
    // }

    //////////////////////////////////
    // Try Oriolo algorithms
    //////////////////////////////////
    oriolo::Oriolo planner(robot, joint_limits, tsr_bounds);

    JointPositions q1{ -1.5, 0.9, 0.9, 1.07333, -0.943054, -0.321943 };

    std::cout << "randRed(...) " << planner.randRed(q1) << "\n";
    std::cout << "randConf() " << planner.randConf() << "\n";
    // std::cout << planner.randConf(regions[0], {1.0, 1.0, 1.0, 1.0, 1.0, 1.0}) << "\n";

    // test ik
    // auto tf_test = robot->fk(q1);
    auto ik_sol = robot->ik(regions[0].tf_nominal, { -1.5, 0.9, 0.9 });
    std::cout << "Found " << ik_sol.size() << " ik solutions\n";
    if (!ik_sol.empty())
    {
        auto q_bias = ik_sol.back();
        std::cout << "Bias: " << q_bias << "\n";
        for (int i = 0; i < 100; ++i)
        {
            auto q_sol = planner.invKin(regions[0], { -1.5, 0.9, 0.9 }, q_bias);
            if (!q_sol.empty())
            {
                std::cout << "invKin1(...) " << q_sol << "\n";
                break;
            }
        }
        for (int i = 0; i < 100; ++i)
        {
            auto q_sol = planner.invKin(regions[0], { -1.5, 0.9, 0.9 });
            if (!q_sol.empty())
            {
                std::cout << "invKin2(...) " << q_sol << "\n";
                break;
            }
        }
        for (int i = 0; i < 100; ++i)
        {
            auto q_sol = planner.randConf(regions[0], q_bias);
            if (!q_sol.empty())
            {
                std::cout << "randConf(...) " << q_sol << "\n";
                break;
            }
        }
    }

    // auto solution = planner.greedy(regions);

    // showPath(solution, rviz, robot);

    auto solution = planner.step(0, 2, q1, regions);
    std::cout << "step() sol size " << solution.size() << "\n";
    //std::cout << "step() " << solution[0] << "\n";

    solution = planner.greedy(regions);
    std::cout << "greedy() sol size " << solution.size() << "\n";

    return 0;
}
