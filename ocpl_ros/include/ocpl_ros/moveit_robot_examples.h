#pragma once

#include <ocpl_ros/moveit_robot.h>

#include <opw_kinematics/opw_parameters.h>

namespace ocpl
{
class PlanarRobot3R : public MoveItRobot
{
  public:
    IKSolution ik(const Transform& tf) override;
};

class PlanarRobotNR : public MoveItRobot
{
    /** \brief Planar robots have 3 base joints for analytical inverse kinematics. **/
    const std::size_t num_base_joints_{ 3 };
    std::string analytical_ik_base_link_;
    std::vector<double> analytical_ik_link_length_;

    void messyHardCodedStuff();

  public:
    PlanarRobotNR(const std::string& tcp_frame = "tool0");
    IKSolution ik(const Transform& tf) override;
    IKSolution ik(const Transform& tf, const std::vector<double>& q_redundant) override;
};

class IndustrialRobot : public MoveItRobot
{
    opw_kinematics::Parameters<double> opw_parameters_;
    std::string group_name_;

    void messyHardCodedStuff();

    /** Copied from MoveIt source code, I should really make all this more generic maybe...
     * http://docs.ros.org/en/melodic/api/moveit_core/html/kinematics__base_8h_source.html#l00618
     * **/
    template <typename T>
    inline bool lookupParam(const std::string& param, T& val, const T& default_val) const
    {
        ros::NodeHandle pnh("~");
        if (pnh.hasParam(group_name_ + "/" + param))
        {
            val = pnh.param(group_name_ + "/" + param, default_val);
            return true;
        }

        if (pnh.hasParam(param))
        {
            val = pnh.param(param, default_val);
            return true;
        }

        ros::NodeHandle nh;
        if (nh.hasParam("robot_description_kinematics/" + group_name_ + "/" + param))
        {
            val = nh.param("robot_description_kinematics/" + group_name_ + "/" + param, default_val);
            return true;
        }

        if (nh.hasParam("robot_description_kinematics/" + param))
        {
            val = nh.param("robot_description_kinematics/" + param, default_val);
            return true;
        }

        val = default_val;

        return false;
    }

  public:
    IndustrialRobot(const std::string& tcp_frame = "tool0");
    IKSolution ik(const Transform& tf) override;
    bool setOPWParameters();
};

}  // namespace ocpl
