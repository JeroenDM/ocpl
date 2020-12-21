#include <ocpl_ros/moveit_robot_examples.h>
#include <ocpl_ros/planar_3r_ik.h>

namespace ocpl
{
IKSolution PlanarRobot3R::ik(const Transform& tf)
{
    // account for fixed transform of tcp
    auto flange_to_tool0 = getLinkFixedRelativeTransform("tool") * getLinkFixedRelativeTransform("tool0");
    auto tf_local = tf * flange_to_tool0.inverse();
    Eigen::Vector3d pos = tf_local.translation();
    Eigen::Vector3d rot = tf_local.rotation().eulerAngles(0, 1, 2);

    // TODO get link lengths from robot model
    std::vector<double> link_length{ 1.0, 1.0, 1.0 };

    return planar_3r_ik(pos[0], pos[1], rot[2], link_length);
}
}  // namespace ocpl
