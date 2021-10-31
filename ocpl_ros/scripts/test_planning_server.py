#!/usr/bin/env python3
from __future__ import print_function

# standard library
import copy
import math

# ros packages
import rospy
import moveit_msgs.srv
import geometry_msgs.msg
import tf_conversions


def rpy_to_quat(rpy):
    # tf_conversions return a numpy array with [x, y, z, w]
    q = tf_conversions.transformations.quaternion_from_euler(*rpy, axes='rxyz')
    return geometry_msgs.msg.Quaternion(*q)


def create_line(a, b, q, num_points):
    """ Create a straight line end-effector path.

    a : Point
    b : Point
    q : orientation of the end-effector along the path

     """
    start_pose = geometry_msgs.msg.Pose()
    start_pose.position = a
    start_pose.orientation = q

    waypoints = [start_pose]
    new_pt = copy.deepcopy(start_pose)
    for i in range(num_points - 1):
        new_pt.position.y += (b.y - a.y) / (num_points - 1)
        waypoints.append(copy.deepcopy(new_pt))

    return waypoints


def create_orientation_constraints(nominal_orientation):
    con = moveit_msgs.msg.OrientationConstraint()

    con.orientation = nominal_orientation
    con.link_name = "tool_tip"
    con.weight = 1.0

    con.absolute_x_axis_tolerance = 0.5
    con.absolute_y_axis_tolerance = 0.0
    con.absolute_z_axis_tolerance = 2.0 * math.pi

    return con


class Client:
    def __init__(self):
        rospy.wait_for_service('ocpl_get_cartesian_path', timeout=5.0)
        self.client = rospy.ServiceProxy(
            'ocpl_get_cartesian_path', moveit_msgs.srv.GetCartesianPath)

    def get_plan(self):
        try:
            request = moveit_msgs.srv.GetCartesianPathRequest()

            q_nominal = rpy_to_quat([0, math.radians(138), math.radians(90)])
            request.waypoints = create_line(geometry_msgs.msg.Point(
                x=0.98, y=-0.5, z=0.02), geometry_msgs.msg.Point(x=0.98, y=0.5, z=0.02), q_nominal, 15)

            request.path_constraints.orientation_constraints = [
                create_orientation_constraints(request.waypoints[0].orientation)]
            response = self.client(request)
            print(response)

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)


if __name__ == "__main__":
    ocpl_client = Client()
    ocpl_client.get_plan()
