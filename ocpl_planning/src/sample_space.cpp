#include <ocpl_planning/sample_space.h>

#include <cassert>

#include <ocpl_planning/types.h>
#include <ocpl_sampling/sampler.h>
#include <ocpl_planning/factories.h>

namespace ocpl
{
SampleSpace::SampleSpace(const Robot& robot, const SampleSpaceSettings& settings, bool debug)
  : robot_(robot), settings_(settings), debug_(debug)
{
    initializeJointSpaceSamplers();
}

void SampleSpace::initializeJointSpaceSamplers()
{
    if (settings_.is_redundant)
    {
        // sampler to generate perturbations on redundant joints with max deviation d around zero
        q_red_local_sampler_ = createLocalSampler(robot_.num_red_dof, settings_.cspace_delta, settings_.sampler_type,
                                                  settings_.redundant_joints_resolution);

        // sampler to generate completely random robot configurations inside the limits
        // at the moment this sampler is never used in the case we do grid sampling
        // but it might be usefull in some cases for RRT like SampleSpaces that use a grid??
        // therefore extend the resolution of the last redundant joint to all the other joints.
        std::vector<int> all_joints_resolution;
        if (!settings_.redundant_joints_resolution.empty())
        {
            all_joints_resolution = settings_.redundant_joints_resolution;
            all_joints_resolution.resize(robot_.num_dof);
            std::fill(all_joints_resolution.begin() + robot_.num_red_dof, all_joints_resolution.end(),
                      settings_.redundant_joints_resolution.back());
        }
        q_sampler_ = createSampler(robot_.joint_limits, settings_.sampler_type, all_joints_resolution);
    }
    else
    {
        q_red_local_sampler_ = createLocalSampler(0, 0, settings_.sampler_type, {});

        // hard coded resolution for non-redundant local sampling of all the joints of the robot
        std::vector<int> all_joints_resolution(robot_.num_dof, 3);
        q_sampler_ = createSampler(robot_.joint_limits, settings_.sampler_type, all_joints_resolution);
    }
}

void SampleSpace::changeSettings(const SampleSpaceSettings& new_settings)
{
    settings_ = new_settings;
    initializeJointSpaceSamplers();
}

void SampleSpace::initializeTaskSpaceSamplers(const std::vector<Bounds> tsr_bounds)
{
    //  sampler to generate random valid end-effector poses paramterised with 6 element vectors
    tsr_sampler_ = createSampler(tsr_bounds, settings_.sampler_type, settings_.tsr_resolution);

    // go through tsr bounds and check were we need to add tolerance
    std::vector<Bounds> tsr_perturbations;
    for (size_t dim{ 0 }; dim < tsr_bounds.size(); ++dim)
    {
        if (tsr_bounds[dim].lower == tsr_bounds[dim].upper)
        {
            tsr_perturbations.push_back({ 0.0, 0.0 });
        }
        else
        {
            tsr_perturbations.push_back({ -settings_.tspace_delta, settings_.tspace_delta });
        }
    }
    tsr_local_sampler_ = createSampler(tsr_perturbations, settings_.sampler_type, settings_.tsr_resolution);
}

/** \brief return random positions for redundant joints. Centered around a bias position.
 *
 * In paper it is described as "generated through a limited random perturbation of q_red_bias".
 * I interpret limited as using the maximum displacement parameter d;
 * **/
JointPositions SampleSpace::sampleRedJoints(const JointPositions& q_bias)
{
    JointPositions q_red_random(robot_.num_red_dof);
    JointPositions perturbation = q_red_local_sampler_->getSample();
    for (size_t i{ 0 }; i < robot_.num_red_dof; ++i)
        q_red_random[i] = clip(q_bias[i] + perturbation[i], robot_.joint_limits[i].lower, robot_.joint_limits[i].upper);
    return q_red_random;
};

JointPositions SampleSpace::sampleRedJoints()
{
    auto q = q_sampler_->getSample();
    return JointPositions(q.begin(), q.begin() + robot_.num_red_dof);
};

JointPositions SampleSpace::sample(const TSR& tsr)
{
    assert(tsr_sampler_ != nullptr);

    // sample c-space for redundant joints
    auto q_red = sampleRedJoints();

    // sample t-space tolerance
    Transform tf = tsr.valuesToPose(tsr_sampler_->getSample());

    // solve inverse kinematics te calculate base joints
    IKSolution sol = robot_.ik(tf, q_red);

    // now select a random ik_solution if tf was reachable
    if (sol.empty())
    {
        return {};
    }
    else
    {
        // TODO select a random one with proper random number generator?
        return sol[rand() % sol.size()];
    }
}

JointPositions SampleSpace::sample(const TSR& tsr, const JointPositions& q_bias)
{
    assert(tsr_local_sampler_ != nullptr);

    // get a biased sample for the redundant joints
    auto q_red = sampleRedJoints(q_bias);

    // // now comes a trickier part, get a biased sample for the end-effector pose
    std::vector<double> v_prev = tsr.poseToValues(robot_.fk(q_bias));

    const auto tsr_bounds = tsr.bounds.asVector();
    for (size_t dim{ 0 }; dim < v_prev.size(); ++dim)
    {
        if (tsr_bounds[dim].lower != tsr_bounds[dim].upper)
        {
            v_prev[dim] = clip(v_prev[dim], tsr_bounds[dim].lower, tsr_bounds[dim].upper);
        }
        else
        {
            v_prev[dim] = 0.0;
        }
    }

    std::vector<double> v_bias = tsr_local_sampler_->getSample();

    // assume the variable that changes between two poses has no tolerance
    // and then ignore the previous value for this variable (v_bias will be zero for that variable)
    // (in most cases this is the x position, as the x-axis is oriented tangent to the path)
    assert(v_prev.size() == v_bias.size());
    for (size_t dim{ 0 }; dim < v_prev.size(); ++dim)
    {
        if (tsr_bounds[dim].lower != tsr_bounds[dim].upper)
        {
            v_bias[dim] += v_prev[dim];
            v_bias[dim] = clip(v_bias[dim], tsr_bounds[dim].lower, tsr_bounds[dim].upper);
        }
    }
    return robot_.biasedIK(tsr.valuesToPose(v_bias), q_red, q_bias, settings_.cspace_delta);
}
}  // namespace ocpl
