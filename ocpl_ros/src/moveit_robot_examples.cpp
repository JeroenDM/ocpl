#include <ocpl_ros/moveit_robot_examples.h>
#include <ocpl_ros/planar_3r_ik.h>

#include <eigen_conversions/eigen_msg.h>
#include <opw_kinematics/opw_kinematics.h>
#include <opw_kinematics/opw_utilities.h>
#include <opw_kinematics/opw_io.h>

#include <ros/ros.h>

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
    assert(num_dof_ > 3);  // this class is for planar redundant robots
    messyHardCodedStuff();
}

void PlanarRobotNR::messyHardCodedStuff()
{
    // guess the base link of the last three joints alternative
    std::size_t link_index = num_dof_ - num_base_joints_;
    analytical_ik_base_link_ =
        joint_model_group_->getActiveJointModels().at(link_index)->getChildLinkModel()->getName();

    // get the link lengths of the last three links
    auto joint_models = joint_model_group_->getActiveJointModels();

    // iterate over the last two active links of the robot
    for (int i{ 2 }; i > 0; --i)
    {
        std::string name = joint_models[num_dof_ - i]->getChildLinkModel()->getName();
        double length = getLinkFixedRelativeTransform(name).translation().norm();
        analytical_ik_link_length_.push_back(length);
    }
    // the last link length is measured towards the flange
    analytical_ik_link_length_.push_back(getLinkFixedRelativeTransform("flange").translation().norm());
}

IKSolution PlanarRobotNR::ik(const Transform& tf)
{
    std::vector<double> zeros(num_dof_ - num_base_joints_, 0.0);
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

/*************************************************
 * Industiral robot (Kuka KR5 hard coded for now)
 * **********************************************/
IndustrialRobot::IndustrialRobot(const std::string& tcp_frame) : MoveItRobot(tcp_frame)
{
    group_name_ = joint_model_group_->getName();
    setOPWParameters();
    messyHardCodedStuff();
}

IKSolution IndustrialRobot::ik(const Transform& pose)
{
    IKSolution joint_poses;

    auto tf_tool0 = pose * tool0_to_tcp_inverse_;

    std::array<double, 6 * 8> sols;
    opw_kinematics::inverse(opw_parameters_, tf_tool0, sols.data());

    // Check the output
    std::vector<double> tmp(6);  // temporary storage for API reasons
    for (int i = 0; i < 8; i++)
    {
        double* sol = sols.data() + 6 * i;
        if (opw_kinematics::isValid(sol))
        {
            opw_kinematics::harmonizeTowardZero(sol);

            // TODO: make this better...
            std::copy(sol, sol + 6, tmp.data());
            // if (isValid(tmp))
            // {
            joint_poses.push_back(tmp);
            // }
        }
    }

    return joint_poses;
}

void IndustrialRobot::messyHardCodedStuff()
{
    // Find the transform between the end-effector tip link
    // and the tool0 reference for the analytical inverse kinematics solver
    auto names = kinematic_model_->getLinkModelNames();
    std::vector<std::string> offset_chain;
    bool tool0_found{ false };
    for (const std::string& name : names)
    {
        std::cout << name << "\n";
        if (tool0_found)
            offset_chain.push_back(name);

        if (name == "tool0")
            tool0_found = true;
    }

    std::cout << "Found offset chain with length: " << offset_chain.size() << "\n";
    for (auto s : offset_chain)
        std::cout << s << ", ";
    std::cout << std::endl;
    
    tool0_to_tcp_ = Transform::Identity();
    for (std::string name : offset_chain)
    {
        tool0_to_tcp_ = tool0_to_tcp_ * getLinkFixedRelativeTransform(name);
    }
    tool0_to_tcp_inverse_ = tool0_to_tcp_.inverse();
    std::cout << "Offset transform: " << tool0_to_tcp_.translation().transpose() << std::endl;
}

bool IndustrialRobot::setOPWParameters()
{
    ROS_INFO_STREAM("Getting kinematic parameters from parameter server.");

    ros::NodeHandle nh;

    std::map<std::string, double> geometric_parameters;
    if (!lookupParam("opw_kinematics_geometric_parameters", geometric_parameters, {}))
    {
        ROS_ERROR_STREAM("Failed to load geometric parameters for ik solver.");
        return false;
    }

    std::vector<double> joint_offsets;
    if (!lookupParam("opw_kinematics_joint_offsets", joint_offsets, {}))
    {
        ROS_ERROR_STREAM("Failed to load joint offsets for ik solver.");
        return false;
    }

    std::vector<int> joint_sign_corrections;
    if (!lookupParam("opw_kinematics_joint_sign_corrections", joint_sign_corrections, {}))
    {
        ROS_ERROR_STREAM("Failed to load joint sign corrections for ik solver.");
        return false;
    }

    opw_parameters_.a1 = geometric_parameters["a1"];
    opw_parameters_.a2 = geometric_parameters["a2"];
    opw_parameters_.b = geometric_parameters["b"];
    opw_parameters_.c1 = geometric_parameters["c1"];
    opw_parameters_.c2 = geometric_parameters["c2"];
    opw_parameters_.c3 = geometric_parameters["c3"];
    opw_parameters_.c4 = geometric_parameters["c4"];

    if (joint_offsets.size() != 6)
    {
        ROS_ERROR_STREAM("Expected joint_offsets to contain 6 elements, but it has " << joint_offsets.size() << ".");
        return false;
    }

    if (joint_sign_corrections.size() != 6)
    {
        ROS_ERROR_STREAM("Expected joint_sign_corrections to contain 6 elements, but it has "
                         << joint_sign_corrections.size() << ".");
        return false;
    }

    for (std::size_t i = 0; i < joint_offsets.size(); ++i)
    {
        opw_parameters_.offsets[i] = joint_offsets[i];
        opw_parameters_.sign_corrections[i] = static_cast<signed char>(joint_sign_corrections[i]);
    }

    ROS_INFO_STREAM("Loaded parameters for ik solver:\n" << opw_parameters_);

    return true;
}

}  // namespace ocpl
