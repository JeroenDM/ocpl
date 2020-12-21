#include <ocpl_tsr/task_space_regions.h>

#include <vector>
#include <Eigen/Geometry>
#include <memory>

#include <ocpl_sampling/sampler.h>
#include <ocpl_sampling/grid_sampler.h>

namespace ocpl
{
TSR::TSR(Transform tf, TSRBounds bounds, SamplerPtr sampler) : tf_nominal_(tf), bounds_(bounds), sampler_(sampler)
{
    sampler_->addDimension(bounds.x.lower, bounds.x.upper);
    sampler_->addDimension(bounds.y.lower, bounds.y.upper);
    sampler_->addDimension(bounds.z.lower, bounds.z.upper);
    sampler_->addDimension(bounds.rx.lower, bounds.rx.upper);
    sampler_->addDimension(bounds.ry.lower, bounds.ry.upper);
    sampler_->addDimension(bounds.rz.lower, bounds.rz.upper);
}

TSR::TSR(Transform tf, TSRBounds bounds, std::shared_ptr<GridSampler> sampler, const std::vector<int>& num_samples)
  : tf_nominal_(tf), bounds_(bounds), sampler_(sampler)
{
    sampler_->addDimension(bounds.x.lower, bounds.x.upper, num_samples[0]);
    sampler_->addDimension(bounds.y.lower, bounds.y.upper, num_samples[1]);
    sampler_->addDimension(bounds.z.lower, bounds.z.upper, num_samples[2]);
    sampler_->addDimension(bounds.rx.lower, bounds.rx.upper, num_samples[3]);
    sampler_->addDimension(bounds.ry.lower, bounds.ry.upper, num_samples[4]);
    sampler_->addDimension(bounds.rz.lower, bounds.rz.upper, num_samples[5]);
}

std::vector<Transform> TSR::getSamples(const int n) const
{
    std::vector<Transform> samples;
    auto tsr_samples = sampler_->getSamples(n);
    for (auto& tsr_sample : tsr_samples)
    {
        samples.push_back(valuesToPose(tsr_sample));
    }
    return samples;
}

Transform TSR::valuesToPose(std::vector<double>& values) const
{
    using Translation = Eigen::Translation3d;
    using AngleAxis = Eigen::AngleAxisd;
    using Vector = Eigen::Vector3d;

    // clang-format off
    Transform t;
      t = Translation(values[0], values[1], values[2]) *
          AngleAxis(values[3], Vector::UnitX()) *
          AngleAxis(values[4], Vector::UnitY()) *
          AngleAxis(values[5], Vector::UnitZ());
    // clang-format on
    return tf_nominal_ * t;
}
}  // namespace ocpl
