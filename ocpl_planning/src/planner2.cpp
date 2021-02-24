#include <ocpl_planning/planner2.h>

#include <limits>

#include <ocpl_planning/factories.h>
#include <ocpl_planning/math.h>
#include <ocpl_planning/containers.h>

namespace ocpl
{
Planner2::Planner2(const std::string& name, const Robot& robot, const Planner2Settings& settings)
  : name_(name), robot_(robot), settings_(settings)
{
    // sampler to generate perturbations on redundant joints with max deviation d around zero
    q_red_local_sampler_ = createLocalSampler(robot.num_red_dof, settings_.DQ_MAX, SamplerType::RANDOM);

    // sampler to generate completely random robot configurations inside the limits
    q_sampler_ = createIncrementalSampler(robot.joint_limits, SamplerType::RANDOM);
}

void Planner2::initializeTaskSpaceSamplers(const std::vector<Bounds> tsr_bounds)
{
    //  sampler to generate random valid end-effector poses paramterised with 6 element vectors
    tsr_sampler_ = createIncrementalSampler(tsr_bounds, SamplerType::RANDOM);

    // go through tsr bounds and check were we need to add tolerance
    std::vector<Bounds> tsr_perturbations;
    has_tolerance_.clear();
    for (size_t dim{ 0 }; dim < tsr_bounds.size(); ++dim)
    {
        if (tsr_bounds[dim].lower == tsr_bounds[dim].upper)
        {
            tsr_perturbations.push_back({ 0.0, 0.0 });
            has_tolerance_.push_back(0);
        }
        else
        {
            tsr_perturbations.push_back({ -settings_.TSR_PERT, settings_.TSR_PERT });
            has_tolerance_.push_back(1);
        }
    }
    tsr_local_sampler_ = createIncrementalSampler(tsr_perturbations, SamplerType::RANDOM);
}

bool Planner2::noColl(const JointPositions& q_from, const JointPositions& q_to) const
{
    // assume q_from is collision free
    // first check goal and quit early if this is in collision
    if (!robot_.isValid(q_to))
    {
        return false;
    }
    // TODO add resolution for collision checking, now hardcoded 3 steps
    const int steps{ 3 };
    for (int step = 1; step < steps - 1; ++step)
    {
        auto q_step = interpolate(q_from, q_to, static_cast<double>(step) / (steps - 1));
        if (!robot_.isValid(q_step))
        {
            return false;
        }
    }
    return true;
}

std::vector<JointPositions> Planner2::sample(size_t waypoint, size_t num_samples) const
{
    assert(!task_.empty());
    assert(tsr_sampler_ != nullptr);
    assert(tsr_local_sampler_ != nullptr);

    auto q_samples = q_sampler_->getSamples(num_samples);
    auto tsr_samples = tsr_sampler_->getSamples(num_samples);

    // solve inverse kinematics te calculate base joints
    std::vector<JointPositions> samples;
    samples.reserve(num_samples);
    // #pragma omp parallel
    // #pragma omp for
    for (size_t i{ 0 }; i < num_samples; ++i)
    {
        Transform tf = task_[waypoint].valuesToPose(tsr_samples[i]);
        JointPositions q_red_sample(q_samples[i].begin(), q_samples[i].begin() + robot_.num_red_dof);
        IKSolution sol = robot_.ik(tf, q_red_sample);
        for (auto q_sol : sol)
        {
            if (robot_.isValid(q_sol))
            {
                samples.push_back(q_sol);
            }
        }
    }

    samples.shrink_to_fit();
    return samples;
}

std::vector<JointPositions> Planner2::sampleIncrementally(size_t waypoint, size_t min_num_samples,
                                                          size_t batch_size) const
{
    assert(!task_.empty());
    assert(tsr_sampler_ != nullptr);
    assert(tsr_local_sampler_ != nullptr);

    std::vector<JointPositions> samples;
    samples.reserve(min_num_samples + batch_size);

    size_t iter{ 0 };
    while (samples.size() < min_num_samples && iter < settings_.MAX_SHOTS)
    {
        auto q_samples = q_sampler_->getSamples(batch_size);
        auto tsr_samples = tsr_sampler_->getSamples(batch_size);

        // solve inverse kinematics te calculate base joints
        // #pragma omp parallel
        // #pragma omp for
        for (size_t i{ 0 }; i < batch_size; ++i)
        {
            Transform tf = task_[waypoint].valuesToPose(tsr_samples[i]);
            JointPositions q_red_sample(q_samples[i].begin(), q_samples[i].begin() + robot_.num_red_dof);
            IKSolution sol = robot_.ik(tf, q_red_sample);
            for (auto q_sol : sol)
            {
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

std::vector<JointPositions> Planner2::sample(size_t waypoint, const JointPositions& q_bias, size_t num_samples) const
{
    assert(!task_.empty());
    assert(tsr_sampler_ != nullptr);
    assert(tsr_local_sampler_ != nullptr);

    // Get a biased sample for the redundant joints
    auto red_sample_perturbations = q_red_local_sampler_->getSamples(num_samples);
    std::vector<JointPositions> q_red_samples;
    q_red_samples.reserve(num_samples);
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
        v_prev = task_[waypoint - 1].poseToValues(robot_.fk(q_bias));
    }
    else  // special case, biased sampling for the first waypoint
    {
        v_prev = task_[0].poseToValues(robot_.fk(q_bias));
    }

    // Now we add a perturbation on v_prev using the local tsr sampler.
    // This sample is then converted to a 3D Transform used later to solve the inverse kinematics.
    auto tsr_samples = tsr_local_sampler_->getSamples(num_samples);
    std::vector<Transform> tf_samples;
    tf_samples.reserve(num_samples);
    for (auto tsr_sample : tsr_samples)
    {
        std::vector<double> v_bias = tsr_sample;
        // assume the variable that changes between two poses has no tolerance
        for (size_t dim{ 0 }; dim < has_tolerance_.size(); ++dim)
        {
            if (has_tolerance_[dim] == 1)
            {
                v_bias.at(dim) += v_prev.at(dim);
            }
        }
        Transform tf = task_[waypoint].valuesToPose(v_bias);
        tf_samples.push_back(tf);
    }

    // solve inverse kinematics te calculate base joints
    std::vector<JointPositions> samples;
    samples.reserve(num_samples);
    // #pragma omp parallel
    // #pragma omp for
    for (size_t i{ 0 }; i < num_samples; ++i)
    {
        IKSolution sol = robot_.ik(tf_samples[i], q_red_samples[i]);
        for (auto q_sol : sol)
        {
            if (norm1Diff(q_sol, q_bias) < settings_.DQ_MAX)
            {
                if (noColl(q_bias, q_sol))
                {
                    samples.push_back(q_sol);
                }
            }
        }
    }
    samples.shrink_to_fit();
    return samples;
}

std::vector<JointPositions> Planner2::updatePath(const std::vector<JointPositions>& path, const JointPositions& q,
                                                 size_t k, size_t k_prev) const
{
    std::vector<JointPositions> new_path = path;
    if (k > k_prev || path.size() == 0)
    {
        new_path.push_back(q);
    }
    else
    {
        for (size_t i{ 0 }; i < (k_prev - k + 1); ++i)
        {
            if (path.empty())
                break;
            else
                new_path.pop_back();  // TODO
        }
        new_path.push_back(q);
    }
    return new_path;
}

std::vector<JointPositions> Planner2::search(BaseContainer& A)
{
    auto qs_start = sample(0, settings_.MAX_SHOTS);
    if (qs_start.empty())
    {
        return {};
    }
    std::cout << "Planner found " << qs_start.size() << " start states.\n";

    // std::vector<Vertice> A;
    // A.reserve(qs_start.size());
    for (JointPositions q : qs_start)
    {
        A.push(std::make_shared<Vertice>(q, 0, 0.0));
    }

    VerticePtr current;
    size_t k{ 0 }, k_prev{ 0 };
    std::vector<JointPositions> path;
    while (!A.empty())
    {
        current = A.pop();
        k = current->waypoint;

        path = updatePath(path, current->q, k, k_prev);
        if (k == (task_.size() - 1))
        {
            return path;
        }

        for (auto q : sample(k + 1, current->q, settings_.MAX_SHOTS))
        {
            A.push(std::make_shared<Vertice>(q, k + 1, norm2Diff(q, current->q)));
        }
        k_prev = k;
    }
    return {};
}

std::vector<std::vector<JointPositions>> Planner2::createRoadMap() const
{
    std::vector<std::vector<JointPositions>> graph_data;
    graph_data.resize(task_.size());

    TSLogger logger;  // Thread save logging

#pragma omp parallel
#pragma omp for
    for (std::size_t k = 0; k < task_.size(); ++k)
    {
        graph_data[k] = sampleIncrementally(k, settings_.MIN_VALID_SAMPLES, 100);
        logger.logWaypoint(k, graph_data[k].size());
    }
    return graph_data;
}

std::vector<JointPositions> Planner2::search_global(BaseContainer& A)
{
    auto graph_data = createRoadMap();

    // convert graph_data into roadmap with vertices
    std::vector<std::vector<VerticePtr>> vertices(task_.size());
    for (size_t k{ 0 }; k < task_.size(); ++k)
    {
        auto qsamples = graph_data[k];
        // std::cout << "Found " << qsamples.size() << " samples for point " << k << "\n";
        vertices[k].reserve(qsamples.size());
        for (auto q : qsamples)
        {
            vertices[k].push_back(std::make_shared<Vertice>(q, k, std::numeric_limits<double>::max()));
        }
    }

    // add start nodes to the container
    for (auto& v : vertices[0])
    {
        v->distance = 0.0;
        A.push(v);
    }

    VerticePtr current;
    size_t k{ 0 }, k_prev{ 0 };
    std::vector<JointPositions> path;
    while (!A.empty())
    {
        current = A.pop();
        k = current->waypoint;

        path = updatePath(path, current->q, k, k_prev);
        if (k == (task_.size() - 1))
        {
            return path;
        }

        for (auto& v : vertices[k + 1])
        {
            if (norm1Diff(v->q, current->q) < settings_.DQ_MAX && noColl(current->q, v->q))
            {
                if (v->visited)
                {
                    v->distance = std::min(v->distance, current->distance + norm2Diff(current->q, v->q));
                }
                else
                {
                    v->visited = true;
                    A.push(v);
                }
            }
        }
        k_prev = k;
    }

    return {};
}

Solution Planner2::solve(const std::vector<TSR>& task)
{
    setTask(task);
    initializeTaskSpaceSamplers(task[0].bounds.asVector());

    Solution solution;
    if (settings_.method == "local_stack")
    {
        StackContainer container;
        solution.path = search(container);
    }
    else if (settings_.method == "local_priority_stack")
    {
        PriorityStackContainer container(task.size());
        solution.path = search(container);
    }
    else
    {
        std::cout << "Planner settings method: " << settings_.method << "\n";
        throw std::runtime_error("Unkown planning method in PlannerSettings");
    }
    // settings_.MAX_SHOTS = 1000;
    // settings_.MIN_VALID_SAMPLES = 2000;
    // settings_.DQ_MAX = 0.5;

    // // QueueContainer container;
    // //   StackContainer container;
    // PriorityStackContainer container(task.size());
    // solution.path = search_global(container);

    // solution.success = solution.path.size() == task.size();
    // solution.cost = calculatePathCost(solution.path);
    return solution;
}

}  // namespace ocpl
