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

Transform TSR::valuesToPose(const std::vector<double>& values) const
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

std::vector<double> TSR::poseToValues(const Transform& tf) const
{
    // values are calculated in nominal frame of this tsr
    Eigen::Isometry3d tf_diff = tf_nominal_.inverse() * tf;
    Eigen::Vector3d pos = tf_diff.translation();
    // Eigen documentation:
    //    The returned angles are in the ranges [0:pi]x[-pi:pi]x[-pi:pi].
    Eigen::Vector3d angles = tf_diff.rotation().eulerAngles(0, 1, 2);

    std::vector<double> values{ pos.x(), pos.y(), pos.z(), angles.x(), angles.y(), angles.z() };
    return values;
}

double TSR::volume(double angle_weight) const
{
    double volume{ 0.0 };
    // position part
    volume += bounds_.x.range();
    volume += bounds_.y.range();
    volume += bounds_.z.range();
    // angular part
    volume += angle_weight * bounds_.rx.range();
    volume += angle_weight * bounds_.ry.range();
    volume += angle_weight * bounds_.rz.range();
    return volume;
}

Eigen::Vector3d minNormEquivalent(const Eigen::Vector3d& angles)
{
    Eigen::Matrix<double, 9, 3> m;
    double x(angles.x()), y(angles.y()), z(angles.z());
    // clang-format off
   m <<  x,  y, z,
        x - M_PI, -y - M_PI, z - M_PI,
        x - M_PI, -y - M_PI, z + M_PI,
        x - M_PI, -y + M_PI, z - M_PI,
        x - M_PI, -y + M_PI, z + M_PI,
        x + M_PI, -y - M_PI, z - M_PI,
        x + M_PI, -y - M_PI, z + M_PI,
        x + M_PI, -y + M_PI, z - M_PI,
        x + M_PI, -y + M_PI, z + M_PI;
    // clang-format on
    // get the index of the row with the lowest norm
    Eigen::VectorXd::Index index;
    m.rowwise().norm().minCoeff(&index);
    return m.row(index);
}

Eigen::Matrix<double, 6, 1> poseDistance(const Transform& tf_ref, const Transform& tf)
{
  const Transform tf_diff = tf_ref.inverse() * tf;
  Eigen::Vector3d angles = minNormEquivalent(tf_diff.rotation().eulerAngles(0, 1, 2));
  Eigen::Matrix<double, 6, 1> d;
  d << tf_diff.translation(), angles;
  return d;
}
}  // namespace ocpl
