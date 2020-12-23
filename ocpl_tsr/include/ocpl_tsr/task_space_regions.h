#pragma once

#include <memory>
#include <vector>

#include <Eigen/Geometry>

#include <ocpl_sampling/sampler.h>
#include <ocpl_sampling/grid_sampler.h>

namespace ocpl
{
typedef Eigen::Isometry3d Transform;

struct Bounds
{
    double lower;
    double upper;
    double range() const
    {
        return upper - lower;
    }
};

struct TSRBounds
{
    Bounds x, y, z;
    Bounds rx, ry, rz;
};

class TSR
{
    Eigen::Isometry3d tf_nominal_;
    TSRBounds bounds_;
    SamplerPtr sampler_;

  public:
    TSR(Transform tf, TSRBounds bounds, SamplerPtr sampler);
    TSR(Transform tf, TSRBounds bounds, std::shared_ptr<GridSampler> sampler, const std::vector<int>& num_samples);
    ~TSR() = default;
    std::vector<Transform> getSamples(const int n = 1) const;

    /** \brief Turn a six vector of position and euler angle values into an end-effector pose.
     *
     * This 'values' vector expresses the local diviation from the nominal task space region frame.
     * Therefore it is pre-multiplied with this nominal pose after conversion, before returning it.
     * */
    Transform valuesToPose(const std::vector<double>& values) const;

    /** \brief Express a pose in the nominal TSR frame and convert it to a six vector of position and euler angles.
     *
     * The reverse of `valuesToPose`.
     * */
    std::vector<double> poseToValues(const Transform& tf) const;
    Transform getNominalPose() const
    {
        return tf_nominal_;
    };

    /** \brief Specific volume metric from the paper
     *
     * Position in meter and angles in randians are weighted equally by default.
     * */
    double volume(double angle_weight = 1.0) const;
};

Eigen::Vector3d minNormEquivalent(const Eigen::Vector3d& angles);
Eigen::Matrix<double, 6, 1> poseDistance(const Transform& tf_ref, const Transform& tf);

}  // namespace ocpl
