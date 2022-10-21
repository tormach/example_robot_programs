from robot_command.rpl import *
set_units("mm", "deg")
waypoint_1 = p[512.6734972000122, 332.86237716674805, 696.8697905540466, 305.75241838, -89.99890888272367, -125.75171662910061]
waypoint_2 = p[785.3748798370361, -45.79196870326996, 569.8965191841125, 0.0, -89.99962609587891, -156.24330551160443]
waypoint_3 = p[658.8226556777954, 289.9807095527649, 415.5462384223938, 209.77703860741997, -40.43829965950539, 5.250951517672397]

import rospy
import os
from math import pi
from random import random
from visualization_msgs.msg import Marker, MarkerArray


class Globals:
    def __init__(self):
        self.pubs = {}
        self.first_run = True
g = Globals()


def register_services():
    g.pubs['markers'] = rospy.Publisher('/pose_marker_array', MarkerArray, queue_size=1, latch=True)


def unregister_services():
    for pub in g.pubs.values():
        pub.unregister()


def create_markers(poses, names):
    units = get_units()
    markers = MarkerArray()
    marker = Marker()
    marker.action = Marker.DELETEALL
    markers.markers.append(marker)
    for i, pose in enumerate(poses):
        pose = pose.to_ros_units(*units)
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "test_poses"
        marker.id = i * 2
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose.to_ros_pose()
        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        marker.color.a = 1.0
        marker.color.r = random()
        marker.color.g = random()
        marker.color.b = random()
        markers.markers.append(marker)
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "test_poses"
        marker.id = i * 2 + 1
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        pose = pose.copy()
        pose.z += 0.03
        marker.pose = pose.to_ros_pose()
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.text = names[i]
        markers.markers.append(marker)
    g.pubs['markers'].publish(markers)


def program():
    if g.first_run:
        register_services()
        g.first_run = False
        create_markers([waypoint_1, waypoint_2, waypoint_3 ], ["waypoint_1", "waypoint_2", "waypoint_3"])
    movej(waypoint_1)
    movej(waypoint_2)
    movej(waypoint_3)


def main():
    try:
        program()
    except Exception as e:
        unregister_services()
        raise e
