#include <ros/ros.h>

#include <vector>
#include <utility>

#include <ocpl_ros/rviz.h>
#include <ocpl_ros/io.h>

#include <ocpl_tsr/task_space_regions.h>
#include <ocpl_sampling/grid_sampler.h>
#include <ocpl_planning/factories.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ocpl_moveit_demo");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::string FILENAME{ "text.csv" };
    if (argc > 1)
    {
        FILENAME = std::string(argv[1]);
        ROS_INFO_STREAM("Visualizing task from file " << FILENAME);
    }

    auto paths = ocpl::readPathsFromCsvFile(FILENAME);

    ocpl::Rviz rviz;
    rviz.clear();

    for (auto task : paths)
    {
        ROS_INFO("Visualizing a path");
        for (auto pt : task)
        {
            rviz.plotPose(pt.tf_nominal);
            ros::Duration(0.01).sleep();
        }
    }
    ros::Duration(0.2).sleep();

    // visualize end-effector tolerance
    for (auto pt : paths[0])
    {
        std::vector<ocpl::Bounds> tsr_bounds;
        for (auto bound : pt.bounds.asVector())
        {
            ROS_INFO_STREAM("Bounds: " << bound.lower << ", " << bound.upper << "\n");
        }
    }

    // ocpl::GridSampler sampler;
    // std::vector<int> num_samples {1, 1, 1, 3, 1, 10};
    // for (std::size_t i{0}; i < 6; ++i)
    // {
    //     sampler.addDimension(tsr_bounds[i].lower, tsr_bounds[i].upper, num_samples[i]);
    // }
    // // some ugly convergence, I need to improve this part of the interface.
    // ocpl::TSRBounds tsr_b2;
    // tsr_b2.fromVector(tsr_bounds);
    // ocpl::TSR waypoint0(raw_task.waypoints.at(0), tsr_b2);

    // for (auto sample : sampler.getSamples())
    // {
    //     auto tf = waypoint0.valuesToPose(sample);
    //     rviz.plotPose(tf);
    //     ros::Duration(0.01).sleep();
    //     std::cout << tf.translation().transpose() << std::endl;
    //     std::cout <<  tf.rotation().eulerAngles(0, 1, 2).transpose() << std::endl;

    // }
    // ros::Duration(0.2).sleep();

    ros::shutdown();
    return 0;
}
