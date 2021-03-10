#include <ocpl_ros/moveit_robot.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/link_model.h>
// #include <moveit/collision_distance_field/collision_detector_allocator_hybrid.h>
// #include <cmath>

#include <Eigen/Dense>
#include <string>
#include <vector>

#include <ocpl_planning/math.h>  // interpolate

namespace ocpl
{
MoveItRobot::MoveItRobot(const std::string& tcp_frame) : tcp_frame_(tcp_frame), num_red_dof_{ 0 }
{
    // load robot model
    robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>("robot_description");
    // robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    kinematic_model_ = robot_model_loader_->getModel();
    ROS_INFO("Model frame: %s", kinematic_model_->getModelFrame().c_str());

    state_storage_.init(kinematic_model_);

    joint_model_group_ = kinematic_model_->getJointModelGroup("manipulator");

    // create planning scene to for collision checking
    planning_scene_.reset(new planning_scene::PlanningScene(kinematic_model_));
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    updatePlanningScene();

    // initDistanceFields();
    // testDistanceFields();

    planning_scene_->printKnownObjects();

    // planning_scene_->getAllowedCollisionMatrixNonConst().setEntry("box_1", "link_3", true);
    // planning_scene_->getAllowedCollisionMatrixNonConst().setEntry("box_1", "link_4", true);
    // planning_scene_->getAllowedCollisionMatrixNonConst().setEntry("box_1", "link_5", true);
    // planning_scene_->getAllowedCollisionMatrixNonConst().setEntry("box_1", "tool", true);

    // auto acm = planning_scene_->getAllowedCollisionMatrix();
    // moveit_msgs::AllowedCollisionMatrix acm_msg;
    // acm.getMessage(acm_msg);

    // ROS_INFO_STREAM("--- Allowed collision matrix ---");
    // std::cout << acm_msg << "\n";
    // // for (auto name : acm.getAllEntryNames())
    // // {
    // //     std::cout << "name: " << name << "\n";
    // // }

    num_dof_ = joint_model_group_->getActiveJointModelNames().size();

    for (auto jm : joint_model_group_->getActiveJointModels())
    {
        joint_position_limits_.push_back(
            { jm->getVariableBounds().at(0).min_position_, jm->getVariableBounds().at(0).max_position_ });
        joint_velocity_limits_.push_back(
            { jm->getVariableBounds().at(0).min_velocity_, jm->getVariableBounds().at(0).max_velocity_ });
    }

    // debug info
    ROS_DEBUG_STREAM("Number of DOFS: " << num_dof_);
    auto joint_names = joint_model_group_->getActiveJointModelNames();
    for (const std::string& name : joint_names)
        ROS_DEBUG_STREAM("Joint name: " << name);
}

// void MoveItRobot::initDistanceFields()
// {
//     //
//     //  ps->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorHybrid::create(), true);
//     planning_scene_->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorHybrid::create(),
//     true);

//     // CHOMP code
//     std::vector<std::string> cd_names;
//     planning_scene_->getCollisionDetectorNames(cd_names);

//     ROS_INFO_STREAM("The following collision detectors are available in the planning scene.");
//     for (std::size_t i = 0; i < cd_names.size(); i++)
//     {
//         ROS_INFO_STREAM(cd_names[i]);
//     }

//     ROS_INFO_STREAM("Active collision detector is: " + planning_scene_->getActiveCollisionDetectorName());

//     hy_world_ = dynamic_cast<const collision_detection::CollisionWorldHybrid*>(
//         planning_scene_->getCollisionWorld(planning_scene_->getActiveCollisionDetectorName()).get());
//     if (!hy_world_)
//     {
//         ROS_WARN_STREAM("Could not initialize hybrid collision world from planning scene");
//         return;
//     }

//     hy_robot_ = dynamic_cast<const collision_detection::CollisionRobotHybrid*>(
//         planning_scene_->getCollisionRobot(planning_scene_->getActiveCollisionDetectorName()).get());
//     if (!hy_robot_)
//     {
//         ROS_WARN_STREAM("Could not initialize hybrid collision robot from planning scene");
//         return;
//     }

//     // END CHOMP CODE
// }

// void MoveItRobot::testDistanceFields()
// {
//     auto state = state_storage_.getAState();
//     state->setToDefaultValues();

//     collision_detection::GroupStateRepresentationPtr gsr_;

//     collision_detection::CollisionRequest req;
//     collision_detection::CollisionResult res;
//     req.group_name = joint_model_group_->getName();
//     ros::WallTime wt = ros::WallTime::now();
//     hy_world_->getCollisionGradients(req, res, *hy_robot_->getCollisionRobotDistanceField().get(), *state,
//                                      &planning_scene_->getAllowedCollisionMatrix(), gsr_);
//     ROS_INFO_STREAM("First coll check took " << (ros::WallTime::now() - wt));
//     // num_collision_points_ = 0;
//     for (size_t i = 0; i < gsr_->gradients_.size(); i++)
//     {
//         collision_detection::GradientInfo& info = gsr_->gradients_[i];
//         std::cout << "value: " << info.joint_name << "\n";
//         std::cout << "value: " << info.collision << "\n";
//         std::cout << "value: " << info.distances.size() << "\n";
//         for (size_t j{ 0 }; j < std::max(info.distances.size(), (size_t)5); ++j)
//         {
//             std::cout << "value: " << info.distances[j] << " and r: " << info.sphere_radii[j] << "\n";
//         }
//     }
// }

const Transform& MoveItRobot::getLinkFixedRelativeTransform(const std::string& frame) const
{
    auto robot_state = state_storage_.getAState();
    return robot_state->getLinkModel(frame)->getJointOriginTransform();
}

const Transform& MoveItRobot::fk(const std::vector<double>& q) const
{
    auto robot_state = state_storage_.getAState();
    robot_state->setJointGroupPositions(joint_model_group_, q);
    return robot_state->getGlobalLinkTransform(tcp_frame_);
}

const Transform& MoveItRobot::fk(const std::vector<double>& q, const std::string& frame) const
{
    auto robot_state = state_storage_.getAState();
    robot_state->setJointGroupPositions(joint_model_group_, q);
    return robot_state->getGlobalLinkTransform(frame);
}

IKSolution MoveItRobot::ik(const Transform& tf)
{
    auto robot_state = state_storage_.getAState();
    double timeout = 0.1;
    robot_state->setToDefaultValues();  // use a deterministic state to initialize IK solver
    bool found_ik = robot_state->setFromIK(joint_model_group_, tf, timeout);
    IKSolution sol;
    if (found_ik)
    {
        std::vector<double> joint_values;
        robot_state->copyJointGroupPositions(joint_model_group_, joint_values);
        sol.push_back(joint_values);
    }
    else
    {
        ROS_INFO_STREAM("Failed to find ik solution.");
    }
    return sol;
}

IKSolution MoveItRobot::ik(const Transform& tf, const std::vector<double>& q_redundant)
{
    double timeout = 0.1;
    auto robot_state = state_storage_.getAState();
    robot_state->setToDefaultValues();  // use a deterministic state to initialize IK solver

    // fill out the redundant joint values that where provided as an extra argument
    static const std::vector<std::string> joint_names = joint_model_group_->getActiveJointModelNames();
    for (std::size_t i{ 0 }; i < q_redundant.size(); ++i)
    {
        robot_state->setJointPositions(joint_names[i], { q_redundant[i] });
    }

    bool found_ik = robot_state->setFromIK(joint_model_group_, tf, timeout);
    IKSolution sol;
    if (found_ik)
    {
        std::vector<double> joint_values;
        robot_state->copyJointGroupPositions(joint_model_group_, joint_values);
        sol.push_back(joint_values);
    }
    else
    {
        ROS_INFO_STREAM("Failed to find ik solution.");
    }
    return sol;
}

Eigen::MatrixXd MoveItRobot::jacobian(const std::vector<double>& q)
{
    auto robot_state = state_storage_.getAState();
    Eigen::Vector3d reference_point(0.0, 0.0, 0.0);
    robot_state->setJointGroupPositions(joint_model_group_, q);
    return robot_state->getJacobian(joint_model_group_);
}

void MoveItRobot::updatePlanningScene()
{
    // I'm not sure yet how this works
    planning_scene_monitor_->requestPlanningSceneState();
    planning_scene_monitor::LockedPlanningSceneRW ps(planning_scene_monitor_);
    ps->getCurrentStateNonConst().update();
    planning_scene_ = ps->diff();
    planning_scene_->decoupleParent();
}

bool MoveItRobot::isInCollision(const std::vector<double>& joint_pose) const
{
    auto robot_state = state_storage_.getAState();
    robot_state->setJointGroupPositions(joint_model_group_, joint_pose);
    // No distance
    // ***********
    bool in_collision = false;

    // ROS_INFO("Checking for collision.");
    // planning_scene_->printKnownObjects(std::cout);

    robot_state->setJointGroupPositions(joint_model_group_, joint_pose);
    in_collision = planning_scene_->isStateColliding(*robot_state);
    return in_collision;

    // With distance calculation
    // *************************
    // collision_detection::CollisionRequest req;
    // req.verbose = false;
    // req.group_name = joint_model_group_->getName();
    // req.distance = true;
    // collision_detection::CollisionResult res;

    // planning_scene_->checkCollision(req, res, *robot_state);

    // std::cout << "CC: " << res.collision << ", dist: " << res.distance << "\n";

    // return res.collision;

    // ros::V_string links;
    // planning_scene_->getCollidingLinks(links, *kinematic_state_);
    // for (auto l : links)
    // {
    //     std::cout << l << "\n";
    // }
}

bool MoveItRobot::isPathColliding(const JointPositions& q_from, const JointPositions& q_to, int steps) const
{
    // std::cout << "isPathColliding -----------------\n";
    for (int step = 0; step < steps; ++step)
    {
        auto q_step = interpolate(q_from, q_to, static_cast<double>(step) / (steps - 1));
        // std::cout << "s: " << static_cast<double>(step) / (steps - 1) << " q: ";
        // for (double v : q_step)
        //     std::cout << v << ", ";
        // std::cout << "\n";
        if (isInCollision(q_step))
        {
            return true;
        }
    }
    return false;
}

void MoveItRobot::plot(moveit_visual_tools::MoveItVisualToolsPtr mvt, std::vector<double>& joint_pose,
                       const rviz_visual_tools::colors& color)
{
    namespace rvt = rviz_visual_tools;
    auto robot_state = state_storage_.getAState();
    robot_state->setJointGroupPositions(joint_model_group_, joint_pose);
    mvt->publishRobotState(robot_state, color);
    mvt->trigger();
}

void MoveItRobot::animatePath(moveit_visual_tools::MoveItVisualToolsPtr mvt, const std::vector<JointPositions>& path)
{
    for (JointPositions q : path)
    {
        if (isInCollision(q))
            plot(mvt, q, rviz_visual_tools::RED);
        else
            plot(mvt, q, rviz_visual_tools::GREEN);
        ros::Duration(0.1).sleep();
    }
}

}  // namespace ocpl
