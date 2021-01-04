#!/usr/bin/env python
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
    pose_s.pose = Pose(position=Vector3(2.0, 0.55, 0),
                       orientation=Quaternion(w=1))

    pose_2 = PoseStamped()
    pose_2.header.frame_id = "world"
    pose_2.pose = Pose(position=Vector3(1.9, -1, 0),
                       orientation=Quaternion(w=1))

    return ["sphere", "sphere"], [pose_s, pose_2], [0.2, 0.4]

def create_case_2_2018():
    small_passage_width = 0.5

    pose_1 = PoseStamped()
    pose_1.header.frame_id = "world"
    pose_1.pose = Pose(position=Vector3(6, -1, 0),
                       orientation=Quaternion(w=1))

    pose_2 = PoseStamped()
    pose_2.header.frame_id = "world"
    pose_2.pose = Pose(position=Vector3(6, small_passage_width + 1, 0),
                       orientation=Quaternion(w=1))

    return ["box", "box"], [pose_1, pose_2], [(4, 2, 1), (4, 2, 1)]

factories = {
    "3r_spheres": create_3r_spheres,
    "case_2_2018": create_case_2_2018
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
