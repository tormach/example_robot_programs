from robot_command.rpl import *
# from machinekit import hal
# import csv
import numpy as np
# import PyKDL
from math import pi
import os
import rospy
from geometry_msgs.msg import Pose as ROSPose
# from tf_conversions import transformations
# from typing import Tuple

from geometry_utils import prepare_observation_pose, prepare_gripper_pose, movel_ros, movej_ros, matrix_to_pose, translate_pose
from geometry_utils import save_poses_to_csv, read_poses_from_csv, rot_z, pose_to_matrix, transform_pose
from tending_settings import TendingSettings, Object

from joint_torque_comparator import JointTorqueComparator
from LidarServiceHandler import LidarServiceHandler

from pneumatic_tools import open_main_gripper, close_main_gripper
from tending_settings import TendingSettings, LidarSettings

torque_comparator = JointTorqueComparator(torque_thresh=20.0)
lidar_handler = LidarServiceHandler()


def refine_pose_icp(initial_pose: ROSPose, settings: TendingSettings):
    SAFETY_LIDAR_OFFSET = 0.015
    x_offset = (settings.workpiece.x / 2) + SAFETY_LIDAR_OFFSET
    min_object_z = initial_pose.position.z + settings.lidar_settings.z_min
    max_object_z = initial_pose.position.z + settings.lidar_settings.z_max

    pose_1, pose_1_ros = prepare_observation_pose(initial_pose=initial_pose,
                                                  x_offset=x_offset,
                                                  z_offset=settings.lidar_settings.z_refined_clearance)
    # TODO: consider using extra movej that is free of near-singularity velocity penalty
    movej(pose_1, velocity_scale = 0.1)
    rospy.logwarn("Checkpoint 1")
    movel(pose_1, velocity = 0.1)
    rospy.logwarn("Checkpoint 2")
    sleep(0.3)

    pose_2, pose_2_ros = prepare_observation_pose(initial_pose=initial_pose,
                                                  x_offset=-x_offset,
                                                  z_offset=settings.lidar_settings.z_refined_clearance)

    pose_1_mat = pose_to_matrix(pose_1_ros)
    pose_2_mat = pose_to_matrix(pose_2_ros)

    positive_twist_angle = pi/6
    negative_twist_angle = -pi/6

    pose_1_twisted = matrix_to_pose(pose_1_mat @ rot_z(positive_twist_angle))
    pose_2_twisted = matrix_to_pose(pose_2_mat @ rot_z(positive_twist_angle))

    pose_1_twisted_negative = matrix_to_pose(pose_1_mat @ rot_z(negative_twist_angle))
    pose_2_twisted_negative = matrix_to_pose(pose_2_mat @ rot_z(negative_twist_angle))

    custom_vel = 0.1
    custom_vel_scale=0.2

    lidar_handler.lidar_on()
    movel(pose_2, velocity = 0.1)
    rospy.logwarn("Checkpoint 3")
    sleep(0.1)

    movej_ros(pose_2_twisted, velocity_scale=custom_vel_scale)
    # joints_now = get_joint_values()
    # joints_now = j[joints_now.j1, joints_now.j2, joints_now.j3, joints_now.j4, joints_now.j5, joints_now.j6 + positive_twist_angle]
    # movej(joints_now, velocity_scale=0.3)

    movel_ros(pose_2_twisted, velocity=custom_vel)
    movel_ros(pose_1_twisted, velocity=custom_vel)
    movej_ros(pose_1_twisted_negative, velocity_scale = custom_vel_scale)

    # joints_now = get_joint_values()
    # joints_now = j[joints_now.j1, joints_now.j2, joints_now.j3, joints_now.j4, joints_now.j5, joints_now.j6 + 2*negative_twist_angle]
    # movej(joints_now, velocity_scale=0.3)
    movel_ros(pose_1_twisted_negative, velocity=custom_vel)
    movel_ros(pose_2_twisted_negative, velocity=custom_vel)

    lidar_handler.lidar_off()

    # TODO:
    movej(pose_2, velocity_scale = custom_vel_scale)
    movel(pose_2, velocity = custom_vel)

    sleep(0.1)
    ros_object_pose = lidar_handler.call_find_closest_rectangle_service(
                                                      min_z=min_object_z,
                                                      max_z=max_object_z,
                                                      cluster_eps=settings.icp_settings.cluster_eps,
                                                      rectangle_width=settings.workpiece.x,
                                                      rectangle_height=settings.workpiece.y,
                                                      initial_pose=initial_pose,
                                                      enable_plotting_data=True,
                                                      save_figure_to_bitmap_headless=True,
                                                      purge_after_processing=True)

    # rpl_refined_pose = Pose().from_ros_pose(ros_refined_pose)

    initial_mat_to_print = pose_to_matrix(initial_pose)
    refined_mat_to_print = pose_to_matrix(ros_object_pose)

    # Format the matrix as a string with aligned columns
    matrix_str = "Initial transformation matrix:\n"
    matrix_str += "\n".join([" ".join([f"{x:10.4f}" for x in row]) for row in initial_mat_to_print])

    # Print the formatted matrix using rospy.logerr
    rospy.logerr("\n" + matrix_str)

    matrix_str = "Refined transformation matrix:\n"
    matrix_str += "\n".join([" ".join([f"{x:10.4f}" for x in row]) for row in refined_mat_to_print])

    # Print the formatted matrix using rospy.logerr
    rospy.logerr("\n" + matrix_str)

    rpl_refined_pose, ros_refined_pose = prepare_gripper_pose(ros_object_pose, x_offset=0.0)

    return rpl_refined_pose, ros_refined_pose


def refine_and_pick_object_lidar(ros_pose: ROSPose, settings: TendingSettings):
    change_tool_frame("lidar_frame")
    set_units("m", "rad", "s")

    # get the norm xy distance from the robot origin
    radius = np.sqrt(ros_pose.position.x**2 + ros_pose.position.y**2)
    pose_rotated = None
    # RADIUS_THRESHOLD = 0.45
    RADIUS_THRESHOLD = 0.55
    if radius > RADIUS_THRESHOLD:
        # movej_ros(ros_pose, velocity_scale=0.3)
        rospy.logwarn("Checkpoint 0A")
        joints_now = get_joint_values()
        joints_now = j[joints_now.j1, joints_now.j2, joints_now.j3, joints_now.j4, joints_now.j5, joints_now.j6 + pi]
        movej(joints_now, velocity_scale=0.3)
        rospy.logwarn("Checkpoint 0B")

        rotation_mat = rot_z(pi)
        ros_pose = transform_pose(ros_pose, rotation_mat)
        pose_rotated = True

    rospy.logerr(f"Refine and pick, ros_pose: position (x={ros_pose.position.x:.2f}, y={ros_pose.position.y:.2f}, z={ros_pose.position.z:.2f})")
    _, ros_pick_pose = refine_pose_icp(initial_pose=ros_pose, settings=settings)
    # if the object is not detected, return

    # TODO: account for the z-bias of the table plane
    rospy.logerr(f"Ros pick pose position (x={ros_pick_pose.position.x:.2f}, y={ros_pick_pose.position.y:.2f}, z={ros_pick_pose.position.z:.2f})")
    z_translation_offset = settings.stack_settings.safe_z_lift - ros_pick_pose.position.z
    ros_above_grasp_pose = translate_pose(ros_pick_pose, [0,0,-z_translation_offset])
    ros_closer_to_grasp_pose = translate_pose(ros_pick_pose, [0,0,-settings.stack_settings.z_right_above])
    ros_torque_compare_pose = translate_pose(ros_pick_pose, [0,0,-settings.stack_settings.z_torque_check])
    ros_grasp_pose = translate_pose(ros_pick_pose, [0,0,-settings.stack_settings.z_grasp])
    # rospy.logerr(f"Refine and pick, ros_grasp_pose: position (x={ros_grasp_pose.position.x:.2f}, y={ros_grasp_pose.position.y:.2f}, z={ros_grasp_pose.position.z:.2f})")
    change_tool_frame("lidar_gripper_frame")
    movel_ros(ros_above_grasp_pose)
    open_main_gripper()
    movel_ros(ros_closer_to_grasp_pose, velocity=0.5)
    torque_comparator.measure_relax_torque()
    movel_ros(ros_torque_compare_pose, velocity=0.05)

    if not torque_comparator.is_torque_in_bounds():
        while True:
            user_input = input(f"Overtorque at grasping. Retry? [Y/n]", default="Y")
            if user_input.lower() == "y":
                refine_and_pick_object_lidar(ros_pose, settings)
                return
            elif user_input.lower() == "n":
                while True:
                    user_input = input(f"Recover to startup pose? [Y/n]", default="Y")
                    if user_input.lower() == "y":
                        movei(z=0.2, vel=0.05)
                        movej(j[0.0, 0.0, 0.0, 0.0, pi/2, -pi/2], velocity_scale=0.2)
                        exit()
                    elif user_input.lower() == "n":
                        exit()
                    else:
                        notify("Invalid input. Type Y or n.", warning=True)
                return
            else:
                notify("Invalid input. Type Y or n.", warning=True)

    movel_ros(ros_grasp_pose, velocity=0.05)
    close_main_gripper()
    movel_ros(ros_above_grasp_pose, velocity = 0.5)

    if pose_rotated:
        joints_now = get_joint_values()
        joints_now = j[joints_now.j1, joints_now.j2, joints_now.j3, joints_now.j4, joints_now.j5, joints_now.j6 - pi]
        movej(joints_now, velocity_scale=0.3)


def radial_scan(layer_number: int, settings: TendingSettings):
    change_tool_frame("lidar_frame")
    stack_height = layer_number * settings.workpiece.z
    lidar_z_position = stack_height + settings.lidar_settings.z_global_clearance
    min_object_z = stack_height + settings.lidar_settings.z_min
    max_object_z = stack_height + settings.lidar_settings.z_max

    set_units("m", "rad", "s")
    movej(j[0.0, 0.0, 0.0, 0.0, pi/2, -pi/2])

    r_start = 0.4
    r_inc = 0.1
    passes = 5

    final_theta = pi/3

    lidar_handler.lidar_off()
    sleep(0.1)
    lidar_handler.call_purge_lidar_data()

    for i in range(passes):
        r = r_start + ( r_inc * i )
        movel(p[r, 0.0, lidar_z_position, pi, 0, 0])

        if i == 0:
            sleep(3)
        else:
            sleep(1)

        val = get_joint_values()

        old_joints = j[val.j1, val.j2, val.j3, val.j4, val.j5, val.j6]
        new_joints = j[val.j1 + final_theta, val.j2, val.j3, val.j4, val.j5, val.j6]

        lidar_handler.lidar_on()
        sleep(0.5)
        movej(new_joints, velocity_scale = 0.05)
        lidar_handler.lidar_off()
        sleep(1)
        movej(old_joints)
        # TODO: improve moves to more effectively swipe in both directions

    object_ros_poses = lidar_handler.call_process_pointclouds(min_z=min_object_z, max_z=max_object_z)
    return object_ros_poses


def fetch_workpiece_from_stack(part_id: int, settings: TendingSettings):
    set_units("m", "rad", "s")
    LIDAR_GLOBAL_FINDINGS_FILE = 'radial_swipe_poses.csv'

    # if poses file exist with global scan result, perform the scan
    if not os.path.exists(LIDAR_GLOBAL_FINDINGS_FILE):
        layer_number = settings.stack_settings.max_layers
        object_ros_poses = radial_scan(layer_number, settings)
        save_poses_to_csv(object_ros_poses)

    object_ros_poses = read_poses_from_csv()
    num_stacks = len(object_ros_poses)
    completed_stack_levels = part_id // num_stacks

    # if one level of picking was completed, the stack is still higher than 1 level, the additional swipe has not been done yet
    if completed_stack_levels == 1 and completed_stack_levels < settings.stack_settings.max_layers and settings.global_scan_after_layer == True:
        # remove csv with previous global scan findings
        os.remove(LIDAR_GLOBAL_FINDINGS_FILE)
        # settings.stack_settings.max_layers -= 1
        layer_number = settings.stack_settings.max_layers - 1
        object_ros_poses = radial_scan(layer_number, settings)
        save_poses_to_csv(object_ros_poses)
        settings.global_scan_after_layer = False

    object_ros_poses = read_poses_from_csv()
    num_stacks = len(object_ros_poses)
    completed_stack_levels = part_id // num_stacks

    stack_level = settings.stack_settings.max_layers - completed_stack_levels

    if stack_level < 1:
        notify("No more objects to pick. Return to start pose?", warning=True)
        set_units("m", "rad", "s")
        movej(j[0.0, 0.0, 0.0, 0.0, pi/2, -pi/2])
        return None

    current_stack_height = stack_level * settings.workpiece.z
    ros_pose = object_ros_poses[part_id % num_stacks]
    ros_pose.position.z = settings.stack_settings.safe_z_lift
    _, ros_pose_pre_initial = prepare_gripper_pose(ros_pose)
    rospy.logwarn("Checkpoint -1A")

    change_tool_frame("lidar_frame")
    movej_ros(ros_pose_pre_initial, velocity_scale=0.3)
    rospy.logwarn("Checkpoint -1B")
    # movel_ros(ros_pose_pre_initial, velocity=0.2)
    ros_pose.position.z = current_stack_height

    refine_and_pick_object_lidar(ros_pose, settings)
    return True


def movei(x=0, y=0, z=0, vel=200):
    # version of movel that keeps abc rotations constant and moves incrementally in any given xyz direction
    current_pose = get_pose()
    x += current_pose.x
    y += current_pose.y
    z += current_pose.z
    movel(p[x, y, z, current_pose.a, current_pose.b, current_pose.c])
