#include <ocpl_ros/moveit_robot_examples.h>
#include <ocpl_ros/planar_3r_ik.h>

namespace ocpl
{
/**********************
 * 3R Robot
 * *******************/
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

/**********************
 * >3 R Robot
 * *******************/
PlanarRobotNR::PlanarRobotNR(const std::string& tcp_frame) : MoveItRobot(tcp_frame)
{
    assert(num_dof_ > 3); // this class is for planar redundant robots
    messyHardCodedStuff();
}

void PlanarRobotNR::messyHardCodedStuff()
{
    // guess the base link of the last three joints alternative
    std::size_t link_index = num_dof_ - num_base_joints_;
    analytical_ik_base_link_ =
        joint_model_group_->getActiveJointModels().at(link_index)->getChildLinkModel()->getName();
    // getting these values from the urdf is possible ...
    analytical_ik_link_length_ = { 1.0, 1.0, 1.0 };
}

IKSolution PlanarRobotNR::ik(const Transform& tf)
{
    std::vector<double> zeros(num_dof_, 0.0);
    return ik(tf, zeros);
}

IKSolution PlanarRobotNR::ik(const Transform& tf, const std::vector<double>& q_fixed)
{
    // set ik based given the fixed joint values q_fixed
    std::vector<double> q_temp(num_dof_, 0.0);
    for (std::size_t i{ 0 }; i < (num_dof_ - num_base_joints_); ++i)
        q_temp[i] = q_fixed[i];

    auto tf_ik_base = fk(q_temp, analytical_ik_base_link_);

    // account ik_base_frame and fixed transform of tcp
    auto flange_to_tool0 = getLinkFixedRelativeTransform("tool") * getLinkFixedRelativeTransform("tool0");
    auto tf_local = tf_ik_base.inverse() * tf * flange_to_tool0.inverse();

    // find the rotation of the last link around the z-axis in the tf_ik_base frame
    Eigen::Vector3d pos = tf_local.translation();
    Eigen::Vector3d rot = tf_local.rotation().eulerAngles(0, 1, 2);

    auto solution = planar_3r_ik(pos[0], pos[1], rot[2], analytical_ik_link_length_);

    // add fixed joint values to all solutions
    // TODO is this inserting slow?
    for (auto& q_sol : solution)
    {
        q_sol.insert(q_sol.begin(), q_fixed.begin(), q_fixed.end());
    }
    return solution;
}
}  // namespace ocpl
