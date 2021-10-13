from robot_command.rpl import *
set_units("mm", "deg")
waypoint_1 = p[546.1747109889984, 165.27676582336426, 250.81384181976318, 180.0, -25.41662167638844, 8.648821730079367]
waypoint_2 = p[638.1007790565491, -288.29818964004517, 394.27489042282104, 180.0, -25.41668488471593, 8.649083240379278]
waypoint_3 = p[589.67596888542175, -239.76671695709229, 225.04563629627228, 184.746071547, 8.121349475578839, -140.97570447381645]
waypoint_5 = p[534.25118923187256, 170.92667520046234, 717.2449231147766, 180.0, -25.36972502549017, 8.570971035889213]
waypoint_6 = p[582.6284945011139, -142.90057122707367, 717.3783183097839, -180.0, -25.326101022116873, 8.658416093469677]
waypoint_4 = p[531.1514735221863, 263.76259326934814, 700.0418901443481, 180.0, -25.40818530680752, 8.660749558927721]
waypoint_7 = p[546.1735188961029, -200.00043511390686, 699.9995112419128, 180.0, -25.40821059987283, 8.660708809148893]
waypoint_8 = p[546.17369771003723, -199.9998539686203, 200.00000298023224, 180.0, -25.408242231459703, 8.660739583601313]
waypoint_9 = p[546.1865663528442, 299.9908924102783, 250.83377957344055, 180.0, -0.005345786522155134, -0.0001306852773692635]

import geometry_msgs.msg
import moveit_commander
scene = moveit_commander.PlanningSceneInterface()


class Globals:
    def __init__(self):
        self.free_moves = True
        self.joint_moves = True
        self.linear_moves = True
        self.notify = True


g = Globals()


def new_box(object_name, pose, size):
    pose_stamped = geometry_msgs.msg.PoseStamped()
    pose_stamped.header.frame_id = 'world'
    pose_stamped.pose = pose.to_ros_pose()
    scene.add_box(object_name, pose_stamped, size=size)


def create_box():
    scene.remove_world_object('box')
    sleep(0.5)
    box_pose = Pose(x=0.550, y=0.0, z=0.3)
    new_box("box", box_pose, [0.2, 0.1, 0.6])
    sleep(1.0)


def remove_box():
    scene.remove_world_object('box')
    sleep(0.5)


def do_free_moves():
    movef(waypoint_1)
    movef(waypoint_2)
    movef(waypoint_3)
    movef(waypoint_1)


def do_joint_moves():
    movej(waypoint_1)
    movej(waypoint_5)
    movej(waypoint_6)
    movej(waypoint_2)
    movej(waypoint_3)
    movej(waypoint_6)
    movej(waypoint_5)
    movej(waypoint_1)


def do_linear_moves():
    movej(waypoint_9)
    movel(waypoint_4, v=0.2)
    movel(waypoint_7, v=0.2)
    movel(waypoint_8, v=0.2)
    movel(waypoint_7, v=0.2)
    movel(waypoint_4, v=0.2)
    movel(waypoint_9, v=0.2)


def main():
    remove_box()

    if g.free_moves:
        if g.notify:
            notify("Automagically navigating around obstacles using free moves. Moves may be unpredictable.", warning=True)
        do_free_moves()

    if g.joint_moves:
        if g.notify:
            notify("Navigating around obstacles using joint moves. Moves are always the same.", warning=True)
        do_joint_moves()

    if g.linear_moves:
        if g.notify:
            notify("Navigating around obstacles using linear moves. Moves are always the same and in a straight line.")
        do_linear_moves()

    remove_box()
