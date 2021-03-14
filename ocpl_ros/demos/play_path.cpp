#include <ros/ros.h>

#include <algorithm>
#include <limits>
#include <chrono>

#include <ocpl_ros/moveit_robot_examples.h>
#include <ocpl_ros/rviz.h>

#include <ocpl_planning/io.h>

using namespace ocpl;

void showPath(const std::vector<JointPositions>& path, Rviz& rviz, std::shared_ptr<MoveItRobot> robot)
{
    for (JointPositions q : path)
    {
        if (robot->isInCollision(q))
            robot->plot(rviz.visual_tools_, q, rviz_visual_tools::RED);
        else
            robot->plot(rviz.visual_tools_, q, rviz_visual_tools::GREEN);
        ros::Duration(0.1).sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ocpl_moveit_demo_rn");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // std::shared_ptr<MoveItRobot> robot = std::make_shared<PlanarRobotNR>();
    std::shared_ptr<MoveItRobot> robot = std::make_shared<IndustrialRobot>();
    Rviz rviz;
    ros::Duration(0.2).sleep();
    rviz.clear();
    ros::Duration(0.2).sleep();

    std::string filename {"last_path.npy"};

    if (argc > 1)
    {
        filename =  std::string(argv[1]);
    }
    std::cout << "Reading path from file: " << filename << "\n";

    auto path = loadPath(filename);

    if (path.empty())
    {
        std::cout << "No path found.\n";
    }
    {
        showPath(path, rviz, robot);
    }

    return 0;
}
