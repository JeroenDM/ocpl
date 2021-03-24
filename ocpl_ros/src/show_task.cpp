#include <ros/ros.h>

#include <vector>
#include <utility>

#include <ocpl_ros/rviz.h>
#include <ocpl_ros/io.h>

#include <ocpl_tsr/task_space_regions.h>
#include <ocpl_sampling/grid_sampler.h>
#include <ocpl_planning/factories.h>

/** Read and under-constrained end-effector path from a list of waypoints in a csv file.
 *
 * A line in a file constains the a bunch of numbers, represented as named vectors below
 *     p, n, t, lower, upper
 * p: 3 numbers, position
 * n: 3 numbers, z-axis, normal along which an axis pointing out of the robot tool should lie
 * t: 3 numbers, x_axis, tangent vector along the path
 * lower: 6 numbers, lower tolerance bound for position and orientation
 * upper: 6 numbers, upper tolerance bound for position and orientation
 *
 *
 * In total, every line contains 21 numbers separated by ','.
 *
 * The lower and upper bound for orientation tolerance can be for any 3-parameter representation
 * and is expressed relative to the nominal pose of the waypoint.
 *
 * */
typedef std::vector<std::pair<double, double>> ToleranceBounds;
struct RawTask
{
    EigenSTL::vector_Isometry3d waypoints;
    std::vector<ToleranceBounds> tolerance;
};

static std::vector<RawTask> readPathFromFile(const std::string& filename)
{
    std::ifstream file(filename);

    if (!file.is_open())
    {
        std::runtime_error("Failed to read file " + filename);
    }

    std::vector<RawTask> tasks;
    EigenSTL::vector_Isometry3d waypoints;
    std::vector<ToleranceBounds> tolerance;
    std::string line;
    while (std::getline(file, line))
    {
        if (line == "")
        {
            if (!waypoints.empty())
            {
                tasks.push_back({ waypoints, tolerance });
                waypoints.clear();
                tolerance.clear();
            }
        }
        else
        {
            std::stringstream stream(line);
            std::string number;
            std::array<double, 21> v;
            std::size_t i{ 0 };
            while (std::getline(stream, number, ','))
            {
                v.at(i) = std::stod(number);
                i++;
            }
            Eigen::Vector3d pos, x_axis, y_axis, z_axis;
            pos << v[0], v[1], v[2];
            z_axis << v[3], v[4], v[5];  // normal vector
            x_axis << v[6], v[7], v[8];  // tangent vector along the path

            x_axis = x_axis.normalized();
            y_axis = z_axis.cross(x_axis).normalized();
            z_axis = z_axis.normalized();

            Eigen::Isometry3d pose;
            pose.matrix().col(0).head<3>() = x_axis;
            pose.matrix().col(1).head<3>() = y_axis;
            pose.matrix().col(2).head<3>() = z_axis;
            pose.matrix().col(3).head<3>() = pos;

            ToleranceBounds bounds(6);
            for (std::size_t i = 0; i < 6; ++i)
            {
                bounds.at(i) = { v[9 + i], v[9 + 6 + i] };
            }

            waypoints.push_back(pose);
            tolerance.push_back(bounds);
        }
    }
    if (!waypoints.empty())
    {
        tasks.push_back({ waypoints, tolerance });
    }

    return tasks;
}

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

    auto raw_tasks = readPathFromFile(ros::package::getPath("ocpl_ros") + "/data/" + FILENAME);

    ocpl::Rviz rviz;
    rviz.clear();

    for (auto task : raw_tasks)
    {
        ROS_INFO("Visualizing a path");
        for (auto tf : task.waypoints)
        {
            rviz.plotPose(tf);
            ros::Duration(0.01).sleep();
        }
    }
    ros::Duration(0.2).sleep();

    // visualize end-effector tolerance
    for (auto task : raw_tasks)
    {
        std::vector<ocpl::Bounds> tsr_bounds;
        for (auto bounds : task.tolerance.at(0))
        {
            ROS_INFO_STREAM("Bounds: " << bounds.first << ", " << bounds.second << "\n");
            tsr_bounds.push_back({ bounds.first, bounds.second });
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
