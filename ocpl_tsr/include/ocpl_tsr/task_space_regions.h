#pragma once

#include <memory>
#include <vector>

#include <Eigen/Geometry>

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

    std::vector<Bounds> asVector() const
    {
        return { x, y, z, rx, ry, rz };
    }

    void fromVector(const std::vector<Bounds>& v)
    {
        assert(v.size() == 6);
        x = v.at(0);
        y = v.at(1);
        z = v.at(2);
        rx = v.at(3);
        ry = v.at(4);
        rz = v.at(5);
    }

    void fromVector(const std::vector<Bounds>& v)
    {
        assert(v.size() == 6);
        x = v.at(0);
        y = v.at(1);
        z = v.at(2);
        rx = v.at(3);
        ry = v.at(4);
        rz = v.at(5);
    }
};

struct TSR
{
    Eigen::Isometry3d tf_nominal;
    TSRBounds bounds;
    bool local_{ true };

    TSR(Transform tf, TSRBounds bounds);
    ~TSR() = default;

    /** \brief Turn a six vector of position and euler angle values into an end-effector pose.
     *
     * This 'values' vector expresses the local deviation from the nominal task space region frame.
     * Therefore it is pre-multiplied with this nominal pose after conversion, before returning it.
     * */
    Transform valuesToPose(const std::vector<double>& values) const;

    /** \brief Express a pose in the nominal TSR frame and convert it to a six vector of position and euler angles.
     *
     * The reverse of `valuesToPose`.
     * */
    std::vector<double> poseToValues(const Transform& tf) const;

    /** \brief Specific volume metric from the paper
     *
     * Position in meter and angles in randians are weighted equally by default.
     * */
    double volume(double angle_weight = 1.0) const;
};

Eigen::Vector3d minNormEquivalent(const Eigen::Vector3d& angles);
Eigen::Matrix<double, 6, 1> poseDistance(const Transform& tf_ref, const Transform& tf);

inline Transform valuesToPose(const std::vector<double>& values)
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
    return t;
}

inline std::vector<double> poseToValues(const Transform& tf)
{
    Eigen::Vector3d pos = tf.translation();
    // Eigen documentation:
    //    The returned angles are in the ranges [0:pi]x[-pi:pi]x[-pi:pi].
    Eigen::Vector3d angles = tf.rotation().eulerAngles(0, 1, 2);

    std::vector<double> values{ pos.x(), pos.y(), pos.z(), angles.x(), angles.y(), angles.z() };
    return values;
}

}  // namespace ocpl
