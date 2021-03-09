#include <ocpl_planning/acro_planner.h>

#include <ocpl_planning/thread_save_logging.h>
#include <ocpl_planning/factories.h>
#include <ocpl_planning/math.h>

namespace ocpl
{
/************************************************************************
 * GLOBAL SAMPLING
 * **********************************************************************/
std::vector<std::function<IKSolution()>> UnifiedPlanner::createGlobalWaypointSamplers(
    const std::vector<TSR>& task_space_regions, const JointLimits& redundant_joint_limits)
{
    std::vector<std::function<IKSolution()>> path_samplers;
    if (settings_.is_redundant)
    {
        for (auto tsr : task_space_regions)
        {
            SamplerPtr t_sampler =
                createSampler(tsr.bounds.asVector(), settings_.sampler_type, settings_.tsr_resolution);
            SamplerPtr c_sampler =
                createSampler(redundant_joint_limits, settings_.sampler_type, settings_.redundant_joints_resolution);
            path_samplers.push_back([tsr, redundant_joint_limits, t_sampler, c_sampler, this]() {
                IKSolution result;
                for (auto tsr_values : t_sampler->getSamples(settings_.t_space_batch_size))
                {
                    for (auto q_red : c_sampler->getSamples(settings_.c_space_batch_size))
                    {
                        for (auto q : robot_.ik(tsr.valuesToPose(tsr_values), q_red))
                        {
                            if (robot_.isValid(q))
                                result.push_back(q);
                        }
                    }
                }

                return result;
            });
        }
    }
    else
    {
        for (const TSR& tsr : task_space_regions)
        {
            SamplerPtr sampler = createSampler(tsr.bounds.asVector(), settings_.sampler_type, settings_.tsr_resolution);
            path_samplers.push_back([tsr, sampler, this]() {
                IKSolution result;
                for (auto tsr_values : sampler->getSamples(settings_.t_space_batch_size))
                {
                    for (auto q : robot_.ik(tsr.valuesToPose(tsr_values), {}))
                    {
                        if (robot_.isValid(q))
                            result.push_back(q);
                    }
                }
                return result;
            });
        }
    }
    return path_samplers;
}

std::vector<std::vector<JointPositions>>
UnifiedPlanner::sampleGlobalIncremental(std::vector<std::function<IKSolution()>> path_samplers)
{
    std::vector<std::vector<JointPositions>> graph_data;
    graph_data.resize(path_samplers.size());

    TSLogger logger;  // Thread save logging

#pragma omp parallel
#pragma omp for
    for (std::size_t i = 0; i < path_samplers.size(); ++i)
    {
        int iters{ 0 };
        std::vector<JointPositions> valid_samples;
        while (iters < settings_.max_iters && valid_samples.size() < settings_.min_valid_samples)
        {
            auto s = path_samplers[i]();

            // add the new joint positions to valid_samples (stl can be ugly...)
            valid_samples.reserve(valid_samples.size() + std::distance(s.begin(), s.end()));
            valid_samples.insert(valid_samples.end(), s.begin(), s.end());

            iters++;
        }
        graph_data[i] = valid_samples;

        logger.logWaypoint(i, valid_samples.size());

        if (iters == settings_.max_iters)
            logger.log("ocpl_planner: maximum number of iterations reached.\n");
    }
    return graph_data;
}

std::vector<std::vector<JointPositions>>
UnifiedPlanner::sampleGlobalGrid(std::vector<std::function<IKSolution()>> path_samplers)
{
    std::vector<std::vector<JointPositions>> graph_data;
    graph_data.resize(path_samplers.size());

    TSLogger logger;  // Thread save logging

#pragma omp parallel
#pragma omp for
    for (std::size_t i = 0; i < path_samplers.size(); ++i)
    {
        graph_data[i] = path_samplers[i]();

        logger.logWaypoint(i, graph_data[i].size());
    }
    return graph_data;
}

/************************************************************************
 * LOCAL SAMPLING
 * **********************************************************************/
std::vector<JointPositions> UnifiedPlanner::sample(size_t waypoint, const JointPositions& q_bias,
                                                   const std::vector<TSR>& task_space_regions)
{
    assert(tsr_sampler_ != nullptr);
    assert(tsr_local_sampler_ != nullptr);

    // Get a biased sample for the redundant joints
    auto red_sample_perturbations = q_red_local_sampler_->getSamples(settings_.c_space_batch_size);
    std::vector<JointPositions> q_red_samples;
    q_red_samples.reserve(settings_.c_space_batch_size);
    for (auto dq : red_sample_perturbations)
    {
        JointPositions q_red(robot_.num_red_dof);
        // make sure the sample stays inside the robot's joint limits
        for (size_t i{ 0 }; i < robot_.num_red_dof; ++i)
        {
            q_red[i] = clip(q_bias[i] + dq[i], robot_.joint_limits[i].lower, robot_.joint_limits[i].upper);
        }
        q_red_samples.push_back(q_red);
    }

    // Get a biased sample for the end-effector pose.
    // First we get the deviation of the nominal pose for the biased sample of the previous waypoint.
    std::vector<double> v_prev;
    if (waypoint > 0)
    {
        v_prev = task_space_regions[waypoint - 1].poseToValues(robot_.fk(q_bias));
    }
    else  // special case, biased sampling for the first waypoint
    {
        v_prev = task_space_regions[0].poseToValues(robot_.fk(q_bias));
    }

    // Now we add a perturbation on v_prev using the local tsr sampler.
    // This sample is then converted to a 3D Transform used later to solve the inverse kinematics.
    auto tsr_samples = tsr_local_sampler_->getSamples(settings_.c_space_batch_size);
    std::vector<Transform> tf_samples;
    tf_samples.reserve(settings_.c_space_batch_size);
    for (auto tsr_sample : tsr_samples)
    {
        std::vector<double> v_bias = tsr_sample;
        std::vector<Bounds> tsr_bounds = task_space_regions[waypoint].bounds.asVector();
        // assume the variable that changes between two poses has no tolerance
        for (size_t dim{ 0 }; dim < v_bias.size(); ++dim)
        {
            if (tsr_bounds[dim].lower != tsr_bounds[dim].upper)
            {
                v_bias[dim] = clip(v_bias[dim] + v_prev[dim], tsr_bounds[dim].lower, tsr_bounds[dim].upper);
            }
        }
        Transform tf = task_space_regions[waypoint].valuesToPose(v_bias);
        tf_samples.push_back(tf);
    }

    // solve inverse kinematics te calculate base joints
    std::vector<JointPositions> samples;
    samples.reserve(settings_.c_space_batch_size);
    // #pragma omp parallel
    // #pragma omp for
    for (size_t i{ 0 }; i < settings_.c_space_batch_size; ++i)
    {
        IKSolution sol = robot_.ik(tf_samples[i], q_red_samples[i]);
        for (auto q_sol : sol)
        {
            if (normInfDiff(q_sol, q_bias) < settings_.cspace_delta)
            {
                // if (noColl(q_bias, q_sol))
                if (robot_.isValid(q_sol))
                {
                    samples.push_back(q_sol);
                }
            }
        }
    }
    samples.shrink_to_fit();
    return samples;
}

std::vector<JointPositions> UnifiedPlanner::sampleLocalIncremental(
    const JointPositions& q_bias, std::function<IKSolution(const JointPositions&)> local_sampler)
{
    int iters{ 0 };
    std::vector<JointPositions> valid_samples;
    while (iters < settings_.max_iters && valid_samples.size() < settings_.min_valid_samples)
    {
        auto s = local_sampler(q_bias);

        // add the new joint positions to valid_samples (stl can be ugly...)
        valid_samples.reserve(valid_samples.size() + std::distance(s.begin(), s.end()));
        valid_samples.insert(valid_samples.end(), s.begin(), s.end());

        iters++;
    }
    if (iters == settings_.max_iters)
    {
        std::cout << "Sampler reached maximum number of iterations.\n";
    }

    return valid_samples;
}
}  // namespace ocpl
