#!/usr/bin/env python
import rospy
import moveit_commander

from geometry_msgs.msg import Vector3, Quaternion, Pose, PoseStamped

# # PATH line in gap
# N = 5
# G = 0.5
# x = G / 2 * np.ones(N)
# y = np.linspace(4, 6, N)
# a = TolerancedNumber(0, 0, np.pi, samples=10)
# path1 = [TrajectoryPt([x[i], y[i], a]) for i in range(N)]

# # collision scene small gap width G
# sc1 = []
# sc1.append(Rectangle(-2, 4, 2, 4, 0)) x. y, dx, dy
# sc1.append(Rectangle( G, 4, 2, 4, 0))

def remove_all_objects(scene):
    for name in scene.get_known_object_names():
        scene.remove_world_object(name)


if __name__ == "__main__":
    rospy.init_node("load_collision_objects")

    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(1.0)

    remove_all_objects(scene)

    # pose_s = PoseStamped()
    # pose_s.header.frame_id = "world"
    # pose_s.pose = Pose(position=Vector3(2.0, 0.55, 0),
    #                    orientation=Quaternion(w=1))

    # scene.add_sphere("sphere1", pose_s, 0.2)

    # pose_2 = PoseStamped()
    # pose_2.header.frame_id = "world"
    # pose_2.pose = Pose(position=Vector3(1.9, -1, 0),
    #                    orientation=Quaternion(w=1))

    # scene.add_sphere("sphere2", pose_2, 0.4)

    small_passage_width = 0.5

    pose_1 = PoseStamped()
    pose_1.header.frame_id = "world"
    pose_1.pose = Pose(position=Vector3(6, -1, 0),
                       orientation=Quaternion(w=1))
    scene.add_box("box1", pose_1, size=(4, 2, 1))

    pose_2 = PoseStamped()
    pose_2.header.frame_id = "world"
    pose_2.pose = Pose(position=Vector3(6, small_passage_width + 1, 0),
                       orientation=Quaternion(w=1))
    scene.add_box("box2", pose_2, size=(4, 2, 1))



    rospy.loginfo("Done!")
