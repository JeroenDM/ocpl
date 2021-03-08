#include <ocpl_planning/quispe.h>

#include <ocpl_planning/types.h>

namespace ocpl
{
namespace quispe
{
PriorityQueue QuispePlanner::getLocalSamples(const TSR& tsr, const JointPositions& q_bias, int num_samples)
{
    // assert(tsr_local_sampler_ != nullptr);

    // // get a biased sample for the redundant joints
    // std::vector<JointPositions> q_red_samples;
    // auto red_samples = q_red_local_sampler_->getSamples(num_samples);
    // for (auto perturbation : red_samples)
    // {
    //     JointPositions q_red_random(robot_.num_red_dof);
    //     for (size_t i{ 0 }; i < robot_.num_red_dof; ++i)
    //         q_red_random[i] =
    //             clip(q_bias[i] + perturbation[i], robot_.joint_limits[i].lower, robot_.joint_limits[i].upper);
    //     q_red_samples.push_back(q_red_random);
    // }

    // std::vector<double> v_prev = tsr.poseToValues(robot_.fk(q_bias));
    // std::vector<Transform> tf_samples;
    // auto tsr_samples = tsr_local_sampler_->getSamples(num_samples);
    // for (auto tsr_sample : tsr_samples)
    // {
    //     // // now comes a trickier part, get a biased sample for the end-effector pose
    //     std::vector<double> v_bias = tsr_sample;
    //     // assume the variable that changes between two poses has no tolerance
    //     for (size_t dim{ 0 }; dim < has_tolerance_.size(); ++dim)
    //     {
    //         if (has_tolerance_[dim] == 1)
    //         {
    //             v_bias.at(dim) += v_prev.at(dim);
    //         }
    //     }
    //     Transform tf = tsr.valuesToPose(v_bias);
    //     tf_samples.push_back(tf);
    // }

    // // compare function
    // auto cmp = [q_bias](const JointPositions& left, const JointPositions& right) {
    //     // std::cout << "cmp: " << q_bias.back() << "\n";
    //     double d_left = norm2Diff(left, q_bias);
    //     double d_right = norm2Diff(right, q_bias);
    //     return d_left > d_right;
    // };

    // // solve inverse kinematics te calculate base joints
    // PriorityQueue samples(cmp);
    // // #pragma omp parallel
    // // #pragma omp for
    // for (size_t i{ 0 }; i < tf_samples.size(); ++i)
    // {
    //     IKSolution sol = robot_.ik(tf_samples[i], q_red_samples[i]);
    //     for (auto q_sol : sol)
    //     {
    //         if (LInfNormDiff2(q_sol, q_bias) < settings_.cspace_delta)
    //         {
    //             if (noColl(q_bias, q_sol))
    //             {
    //                 // std::unique_lock<std::mutex> lock(priority_queue_mutex_);
    //                 samples.push(q_sol);
    //             }
    //         }
    //     }
    // }
    // return samples;
}

JointPositions QuispePlanner::findChildren(const TSR& tsr, const JointPositions& q_bias, int num_samples)
{
    // JointPositions q_child{};
    // size_t iters{ 0 };
    // bool success{ false };
    // while (!success && iters < settings_.max_iters)
    // {
    //     q_child = sample(tsr, q_bias);
    //     if (!q_child.empty())
    //     {
    //         success = true;
    //     }
    // }
    // return q_child;
}

std::vector<JointPositions> OrioloPlanner::greedy2(const std::vector<TSR>& task)
{
    // initializeTaskSpaceSamplers(task[0].bounds.asVector());
    // std::vector<JointPositions> path;
    // size_t iters{ 0 };
    // bool success{ false };
    // JointPositions q_start{}, q_current{};

    // // try for different start configs until success
    // while (!success && iters < settings_.max_iters)
    // {
    //     std::cout << "Start config iterations " << iters << "\n";
    //     q_start = sample(task[0]);
    //     if (!q_start.empty())
    //     {
    //         path = { q_start };
    //         q_current = q_start;

    //         // depth first with only a single child for every node
    //         for (size_t i{ 1 }; i < task.size(); ++i)
    //         {
    //             size_t iters_2{ 0 };
    //             bool success_2{ false };
    //             while (!success_2 && iters_2 < settings_.max_iters)
    //             {
    //                 std::cout << "Find next config iteration " << iters_2 << "\n";
    //                 auto local_samples = getLocalSamples(task[i], path.back(), 1000);
    //                 if (!local_samples.empty())
    //                 {
    //                     // find a collision free connection
    //                     path.push_back(local_samples.top());
    //                     // while (!success_2 && !local_samples.empty())
    //                     // {
    //                     //     std::cout << "Local samples collision checking left: " << local_samples.size() <<
    //                     "\n";
    //                     //     auto q_try = local_samples.top();
    //                     //     local_samples.pop();
    //                     //     if (noColl(path.back(), q_try))
    //                     //     {
    //                     //         path.push_back(q_try);
    //                     //         std::cout << "cost: " << LInfNormDiff2(path[i], path[i - 1]) << "\n";
    //                     //         success_2 = true;
    //                     //     }
    //                     // }
    //                 }
    //                 iters_2++;
    //             }
    //             if (!success_2)
    //             {
    //                 break;
    //             }
    //         }
    //         if (path.size() == task.size())
    //         {
    //             success = true;
    //         }
    //     }
    //     iters++;
    // }
    // return path;
}

}  // namespace quispe
}  // namespace ocpl
