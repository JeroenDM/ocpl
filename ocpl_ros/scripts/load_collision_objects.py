#!/usr/bin/env python
import rospy
import moveit_commander

from geometry_msgs.msg import Vector3, Quaternion, Pose, PoseStamped


def remove_all_objects(scene):
    for name in scene.get_known_object_names():
        scene.remove_world_object(name)


if __name__ == "__main__":
    rospy.init_node("load_collision_objects")

    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(1.0)

    remove_all_objects(scene)

    pose_s = PoseStamped()
    pose_s.header.frame_id = "world"
    pose_s.pose = Pose(position=Vector3(2.0, 0.55, 0),
                       orientation=Quaternion(w=1))

    scene.add_sphere("sphere1", pose_s, 0.2)

    pose_2 = PoseStamped()
    pose_2.header.frame_id = "world"
    pose_2.pose = Pose(position=Vector3(1.9, -1, 0),
                       orientation=Quaternion(w=1))

    scene.add_sphere("sphere2", pose_2, 0.4)

    rospy.loginfo("Done!")
