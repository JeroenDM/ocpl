#!/usr/bin/env python3
import sys
import rospy
import moveit_commander

from geometry_msgs.msg import Vector3, Quaternion, Pose, PoseStamped


def remove_all_objects(scene):
    for name in scene.get_known_object_names():
        scene.remove_world_object(name)


def create_3r_spheres():
    pose_s = PoseStamped()
    pose_s.header.frame_id = "world"
    pose_s.pose = Pose(position=Vector3(2.0, 0.55, 0), orientation=Quaternion(w=1))

    pose_2 = PoseStamped()
    pose_2.header.frame_id = "world"
    pose_2.pose = Pose(position=Vector3(1.9, -1, 0), orientation=Quaternion(w=1))

    return ["sphere", "sphere"], [pose_s, pose_2], [0.2, 0.4]

def create_case_1_2018():
    poses = [PoseStamped(), PoseStamped(), PoseStamped(), PoseStamped(), PoseStamped()]
    for pose in poses:
        pose.header.frame_id = "world"

    poses[0].pose = Pose(position=Vector3(1.5, 1.75, 0), orientation=Quaternion(w=1))
    poses[1].pose = Pose(position=Vector3(3.5, 2.1, 0), orientation=Quaternion(w=1))
    poses[2].pose = Pose(position=Vector3(2, 3.45, 0), orientation=Quaternion(w=1))
    poses[3].pose = Pose(position=Vector3(0.1, 2.1, 0), orientation=Quaternion(w=1))
    poses[4].pose = Pose(position=Vector3(0.6, 1.25, 0), orientation=Quaternion(w=1))

    return (
        ["box"] * 5,
        poses,
        [(1, 1.5, 1), (1, 2.2, 1), (4, 0.5, 1), (0.2, 2.2, 1), (0.8, 0.5, 1)],
    )

def create_case_2_2018():
    small_passage_width = 0.5

    pose_1 = PoseStamped()
    pose_1.header.frame_id = "world"
    pose_1.pose = Pose(position=Vector3(6, -1, 0), orientation=Quaternion(w=1))

    pose_2 = PoseStamped()
    pose_2.header.frame_id = "world"
    pose_2.pose = Pose(
        position=Vector3(6, small_passage_width + 1, 0), orientation=Quaternion(w=1)
    )

    # pose_3 = PoseStamped()
    # pose_3.header.frame_id = "world"
    # pose_3.pose = Pose(
    #     position=Vector3(2, -1, 0), orientation=Quaternion(w=1)
    # )

    return ["box", "box"], [pose_1, pose_2], [(4, 2, 1), (4, 2, 1)]
    # return ["box", "box", "box"], [pose_1, pose_2, pose_3], [(4, 2, 1), (4, 2, 1), (1, 1, 1)]


def create_case_3_2018():
    poses = [PoseStamped(), PoseStamped(), PoseStamped(), PoseStamped(), PoseStamped()]
    for pose in poses:
        pose.header.frame_id = "world"

    poses[0].pose = Pose(position=Vector3(2.5, -1.1, 0), orientation=Quaternion(w=1))
    poses[1].pose = Pose(position=Vector3(2.5, 3.1, 0), orientation=Quaternion(w=1))
    poses[2].pose = Pose(position=Vector3(1.1, 0.25, 0), orientation=Quaternion(w=1))
    poses[3].pose = Pose(position=Vector3(2.6, 2.0, 0), orientation=Quaternion(w=1))
    poses[4].pose = Pose(position=Vector3(4.1, 0.25, 0), orientation=Quaternion(w=1))

    return (
        ["box"] * 5,
        poses,
        [(5, 0.2, 1), (5, 0.2, 1), (0.2, 2.5, 1), (0.2, 2, 1), (0.2, 2.5, 1)],
    )

def create_case_teapot():
    names = ["box"] * 4
    poses = [PoseStamped(), PoseStamped(), PoseStamped(), PoseStamped()]
    for pose in poses:
        pose.header.frame_id = "world"

    poses[0].pose = Pose(position=Vector3(0.9, 0.0, 1.1), orientation=Quaternion(w=1))
    poses[1].pose = Pose(position=Vector3(0.9, 0.0, 0.25), orientation=Quaternion(w=1))
    poses[2].pose = Pose(position=Vector3(0.9, -0.475, 0.8), orientation=Quaternion(w=1))
    poses[3].pose = Pose(position=Vector3(0.9, 0.475, 0.8), orientation=Quaternion(w=1))

    sizes = [None] * 4
    sizes[0] = (0.4, 1.0, 0.05)
    sizes[1] = (0.4, 1.0, 0.5)
    sizes[2] = (0.4, 0.05, 0.6)
    sizes[3] = (0.4, 0.05, 0.6)

    return (
        names,
        poses,
        sizes,
    )


factories = {
    "3r_spheres": create_3r_spheres,
    "case_1_2018": create_case_1_2018,
    "case_2_2018": create_case_2_2018,
    "case_3_2018": create_case_3_2018,
    "teapot": create_case_teapot,
}

if __name__ == "__main__":
    rospy.init_node("load_collision_objects")

    default_scene = "3r_spheres"
    selected_scene = ""

    if len(sys.argv) < 2:
        print("Using default collision scene {}".format(default_scene))
        selected_scene = default_scene
    else:
        if sys.argv[1] in factories:
            selected_scene = sys.argv[1]
        else:
            rospy.loginfo("Could find a scene with name {}".format(sys.argv[1]))

    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(1.0)

    remove_all_objects(scene)

    types, poses, sizes = factories[selected_scene]()
    for i, t, p, s in zip(range(len(poses)), types, poses, sizes):
        if t == "box":
            scene.add_box("box_{}".format(i), p, size=s)
        elif t == "sphere":
            scene.add_sphere("sphere_{}".format(i), p, s)

    rospy.loginfo("Done!")
