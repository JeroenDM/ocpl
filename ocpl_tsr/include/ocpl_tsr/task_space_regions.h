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
    Transform valuesToPose(std::vector<double>& values) const;
    Transform getNominalPose() const
    {
        return tf_nominal_;
    };
};
}  // namespace ocpl
