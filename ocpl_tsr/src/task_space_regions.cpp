#include <ocpl_tsr/task_space_regions.h>

#include <vector>
#include <Eigen/Geometry>
#include <memory>

namespace ocpl
{
TSR::TSR(Transform tf, TSRBounds bounds) : tf_nominal(tf), bounds(bounds)
{
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
    return tf_nominal * t;
}

std::vector<double> TSR::poseToValues(const Transform& tf) const
{
    // values are calculated in nominal frame of this tsr
    Eigen::Isometry3d tf_diff = tf_nominal.inverse() * tf;
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
    volume += bounds.x.range();
    volume += bounds.y.range();
    volume += bounds.z.range();
    // angular part
    volume += angle_weight * bounds.rx.range();
    volume += angle_weight * bounds.ry.range();
    volume += angle_weight * bounds.rz.range();
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
