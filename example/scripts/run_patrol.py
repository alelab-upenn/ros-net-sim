#!/usr/bin/env python

import actionlib
import copy
from geometry_msgs.msg import Pose
from example.msg import WaypointNavigationAction, WaypointNavigationGoal
from math import cos, sin, pi
import numpy as np
import rospy


def fetch_param_warn(name, default_value):
    try:
        value = rospy.get_param(name)
    except KeyError:
        rospy.logwarn('failed to fetch param \'%s\' from parameter server; using a default value of %s' % (name, str(default_value)))
        return default_value
    return value


def circle(rad, theta):
    pts = np.zeros((theta.size,2))
    pts[:,0] = rad*np.cos(theta)
    pts[:,1] = rad*np.sin(theta)
    return pts


if __name__ == '__main__':

    robot_count = 2
    waypoint_count = 30
    patrol_radius = fetch_param_warn('/patrol_radius', 55.0)  # meters

    #
    # generate circle patrol points
    #

    # elipse points
    circle_points = []
    for i in range(robot_count):
        theta = np.linspace(0.0, 2.0*pi, num=waypoint_count) + 2.0*pi*i/float(robot_count)
        circle_points += [circle(patrol_radius, theta)]

    #
    # send goals to robots
    #

    try:
        rospy.init_node('patrol_node')

        clients = []
        for i in range(robot_count):
            name = '/tb%d/nav_base_nodelet' % (i+1)
            clients += [actionlib.SimpleActionClient(name, WaypointNavigationAction)]
            rospy.loginfo('waiting for %s to start' % (name))
            clients[i].wait_for_server()

        for i in range(robot_count):
            goal = WaypointNavigationGoal()
            goal.header.frame_id = 'world'
            goal.header.stamp = rospy.get_rostime()

            for j in range(waypoint_count):
                pose = Pose()
                pose.position.x = circle_points[i][j,0]
                pose.position.y = circle_points[i][j,1]
                pose.orientation.w = 1.0

                goal.waypoints += [copy.deepcopy(pose)]

            clients[i].send_goal(goal)

        for i in range(robot_count):
            rospy.loginfo('waiting for tb%s to complete' % (i+1))
            clients[i].wait_for_result()

    except rospy.ROSInterruptException:
        pass
