from robot_command.rpl import *
set_units("mm", "deg")

tool_offset = p[77.47, 0, 0, 180.0, 0, 0]

import os
import geometry_msgs.msg
import moveit_commander

chainsaw_stl = os.path.join('.', 'saw-collision.stl')
scene = moveit_commander.PlanningSceneInterface()


def new_mesh(mesh_name, path, pose, scale=1.0):
    mesh_pose = geometry_msgs.msg.PoseStamped()
    mesh_pose.header.frame_id = 'tool0'
    mesh_pose.pose = pose.to_ros_pose()
    scene.add_mesh(mesh_name, mesh_pose, filename=path, size=(
            scale, scale, scale
        ))
    scene.attach_mesh(
        'tool0',
        mesh_name, touch_links=['link_5', 'link_6']
    )


def new_chainsaw():
    units = get_units()
    new_mesh('saw', chainsaw_stl, tool_offset.to_ros_units(*units), scale=0.001)


def main():
    scene.remove_attached_object('tool0')
    scene.remove_world_object('saw')
    sleep(1.0)
    new_chainsaw()
    exit()
