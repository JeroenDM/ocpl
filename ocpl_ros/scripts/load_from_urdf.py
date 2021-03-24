#!/usr/bin/env python
import sys
import rospy
import rospkg
import urdfpy
import moveit_commander

from geometry_msgs.msg import Vector3, Quaternion, Pose, PoseStamped

from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive
from rosgraph.names import ns_join

REL_WORK_PATH = "/urdf/work/"


def numpy_to_pose(arr):
    """ Numpy 4x4 array to geometry_msg.Pose
    Code from: https://github.com/eric-wieser/ros_numpy
    TODO move this to some utility module if I have one.
    """
    from tf import transformations
    assert arr.shape == (4, 4)

    trans = transformations.translation_from_matrix(arr)
    quat = transformations.quaternion_from_matrix(arr)

    return Pose(position=Vector3(*trans), orientation=Quaternion(*quat))


def remove_all_objects(scene):
    for aco in scene.get_attached_objects():
        scene.remove_attached_object(aco)
        scene.remove_world_object(aco)
    for name in scene.get_known_object_names():
        scene.remove_world_object(name)


def parse_urdf_file(package_name, work_name):
    """ Convert urdf file (xml) to python dict.
    Using the urdfpy package for now.
    Using the xml package from the standard library could be
    easier to understand. We can change this in the future
    if it becomes a mess.
    """
    rospack = rospkg.RosPack()

    filepath = rospack.get_path(package_name)
    filepath += REL_WORK_PATH

    urdf = urdfpy.URDF.load(filepath + work_name + ".urdf")

    d = {"links": {}, "joints": {}}

    for link in urdf.links:
        if link.name == "world" or link.name == "work":
            continue
        else:
            d["links"][link.name] = parse_link(link, filepath)

    for joint in urdf.joints:
        p = PoseStamped()
        p.header.frame_id = joint.parent
        p.pose = numpy_to_pose(joint.origin)

        d["joints"][joint.name] = {
            "pose": p,
            "parent": joint.parent,
            "child": joint.child
        }
    return d


def parse_link(link, mesh_path):
    """ Assume a link has only a single collision object.
        Assume this collision object is a box.
        Assume the link named "world" has no collision objects.
    link: a urdfpy.urdf.Link object
    mesh_path: absolute path of the folder where we have to fine the stl files
    """
    assert len(link.collisions) == 1
    assert link.name != "world"
    assert link.name != "work"
    collision = link.collisions[0]
    if collision.geometry.box is not None:
        data = {"type": "box", "size": link.collisions[0].geometry.box.size}
    elif collision.geometry.cylinder is not None:
        data = {"type": "cylinder", "radius": link.collisions[0].geometry.cylinder.radius,
                "length": link.collisions[0].geometry.cylinder.length}
    elif collision.geometry.mesh is not None:
        data = {
            "type": "mesh",
            "filename": mesh_path + collision.geometry.mesh.filename,
            "scale": collision.geometry.mesh.scale
        }
    else:
        raise Exception(
            "URDF link collision geometry is not a mesh, box or cylinder.")

    return data


def make_cylinder(name, pose, height, radius):
    co = CollisionObject()
    co.operation = CollisionObject.ADD
    co.id = name
    co.header = pose.header
    cylinder = SolidPrimitive()
    cylinder.type = SolidPrimitive.CYLINDER
    cylinder.dimensions = [height, radius]
    co.primitives = [cylinder]
    co.primitive_poses = [pose.pose]
    return co


def publish_parsed_urdf(parsed_urdf, scene, ap, attach=False):
    """ Publish link geometry for every joint's child. """
    for name, joint in parsed_urdf["joints"].items():
        # get the child link data
        link = parsed_urdf["links"][joint["child"]]

        # publish the child links collision geometry
        if link["type"] == "box":
            if attach:
                scene.attach_box(
                    "table", joint["child"], joint["pose"], link["size"], ["table"])
            else:
                scene.add_box(
                    joint["child"],
                    joint["pose"],
                    link["size"]
                )
        elif link["type"] == "cylinder":
            if attach:
                aco = AttachedCollisionObject()
                aco.touch_links = ["table"]
                aco.object = make_cylinder(joint["child"],
                                           joint["pose"],
                                           link["length"],
                                           link["radius"])
                aco.link_name = "table"
                print(aco.object)
                ap.publish(aco)
            else:
                scene.add_cylinder(
                    joint["child"],
                    joint["pose"],
                    link["length"],
                    link["radius"]
                )

        else:
            scene.add_mesh(
                joint["child"],
                joint["pose"],
                link["filename"],
                link["scale"]
            )


# def attach_box(self, link, name, pose=None, size=(1, 1, 1), touch_links=[]):
#     aco = AttachedCollisionObject()
#     if pose is not None:
#         aco.object = self.__make_box(name, pose, size)
#     else:
#         aco.object = self.__make_existing(name)
#     aco.link_name = link
#     if len(touch_links) > 0:
#         aco.touch_links = touch_links
#     else:
#         aco.touch_links = [link]
#     self.__submit(aco, attach=True)

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Two argument required:")
        print("<setup_name> <work_name>:\tpublish urdf file <work_name>.urdf.")
        exit()
    else:
        package_name = sys.argv[1] + "_support"
        work_name = sys.argv[2]

    rospy.init_node("publish_work")
    aco_pub = rospy.Publisher(ns_join(
        '', 'attached_collision_object'), AttachedCollisionObject, queue_size=100)

    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(1.0)  # wait for the above things to setup

    remove_all_objects(scene)

    work = parse_urdf_file(package_name, work_name)
    publish_parsed_urdf(work, scene, aco_pub, attach=False)

    print("Done!")
