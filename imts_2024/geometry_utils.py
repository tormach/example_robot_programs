from robot_command.rpl import *

import csv

from geometry_msgs.msg import Pose as ROSPose
import tf.transformations as tf
from tf.transformations import euler_from_matrix, quaternion_from_euler
import numpy as np
from math import pi

def rot_z(theta):
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([
        [c, -s, 0, 0],
        [s,  c, 0, 0],
        [0,  0, 1, 0],
        [0,  0, 0, 1]
    ])

def rpl_pose_to_matrix(rpl_pose):
    """
    Convert a geometry_msgs/Pose message to a 4x4 transformation matrix.

    Args:
    pose (geometry_msgs.msg.Pose): The ROS Pose message

    Returns:
    numpy.ndarray: 4x4 transformation matrix
    """
    pose = rpl_pose.to_ros_pose()
    matrix = pose_to_matrix(pose)
    return matrix

def pose_to_matrix(pose):
    """
    Convert a geometry_msgs/Pose message to a 4x4 transformation matrix.

    Args:
    pose (geometry_msgs.msg.Pose): The ROS Pose message

    Returns:
    numpy.ndarray: 4x4 transformation matrix
    """
    # Extract the translation
    translation = [pose.position.x, pose.position.y, pose.position.z]

    # Extract the quaternion
    quaternion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

    # Create the transformation matrix
    matrix = tf.concatenate_matrices(
        tf.translation_matrix(translation),
        tf.quaternion_matrix(quaternion)
    )

    return matrix

def matrix_to_pose(matrix):
    """
    Convert a 4x4 transformation matrix to a geometry_msgs/Pose message.

    Args:
    matrix (numpy.ndarray): 4x4 transformation matrix

    Returns:
    geometry_msgs.msg.Pose: The equivalent ROS Pose message
    """
    # Ensure the input is a 4x4 numpy array
    if not isinstance(matrix, np.ndarray) or matrix.shape != (4, 4):
        raise ValueError("Input must be a 4x4 numpy array")

    # Extract the translation
    translation = matrix[:3, 3]

    # Extract the rotation matrix
    rotation_matrix = matrix[:3, :3]

    # Convert rotation matrix to euler angles
    roll, pitch, yaw = euler_from_matrix(rotation_matrix)

    # Convert euler angles to quaternion
    quaternion = quaternion_from_euler(roll, pitch, yaw)

    # Create the Pose message
    pose = ROSPose()
    pose.position.x = translation[0]
    pose.position.y = translation[1]
    pose.position.z = translation[2]
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]

    return pose


def transform_pose(A: ROSPose, B: np.ndarray) -> ROSPose:
    """
    Transform a ROS1 ROSPose using a 4x4 homogeneous transformation matrix.

    Args:
    A (geometry_msgs.msg.ROSPose): Input ROS1 ROSPose
    B (numpy.ndarray): 4x4 homogeneous transformation matrix

    Returns:
    geometry_msgs.msg.ROSPose: Transformed ROS1 ROSPose
    """
    # Convert ROS1 ROSPose A to 4x4 homogeneous transformation matrix AMat
    translation = [A.position.x, A.position.y, A.position.z]
    rotation = [A.orientation.x, A.orientation.y, A.orientation.z, A.orientation.w]
    AMat = tf.concatenate_matrices(
        tf.translation_matrix(translation),
        tf.quaternion_matrix(rotation)
    )

    # Perform matrix multiplication to get C
    C = np.dot(AMat, B)

    # Extract translation and rotation from C
    translation = tf.translation_from_matrix(C)
    quaternion = tf.quaternion_from_matrix(C)

    # Create and return the new ROS1 ROSPose CPose
    CPose = ROSPose()
    CPose.position.x, CPose.position.y, CPose.position.z = translation
    CPose.orientation.x, CPose.orientation.y, CPose.orientation.z, CPose.orientation.w = quaternion

    return CPose


def translate_pose(pose: ROSPose, translation: list) -> ROSPose:
    """
    Translate a ROS Pose by a given translation vector.

    Args:
    pose (geometry_msgs.msg.Pose): Input ROS Pose
    translation (list): Translation vector [x, y, z]

    Returns:
    geometry_msgs.msg.Pose: Translated ROS Pose
    """
    # Create a 4x4 homogeneous transformation matrix from the translation vector
    translation_matrix = tf.translation_matrix(translation)

    # Use the transform_pose function to apply the transformation

    translated_pose = transform_pose(pose, translation_matrix)

    return translated_pose

def prepare_gripper_pose(initial_pose: ROSPose, x_offset=0.0):
    # changes workpiece orientation to match gripper frame orientation
    rotation_mat = np.array([
        [0, -1, 0, 0],
        [-1, 0, 0, 0],
        [0, 0, -1, 0],
        [0, 0, 0, 1]
    ])

    ros_pose_rotated = transform_pose(initial_pose, rotation_mat)
    rpl_pose_rotated = Pose.from_ros_pose(ros_pose_rotated)
    return rpl_pose_rotated, ros_pose_rotated

def prepare_observation_pose(initial_pose: ROSPose, x_offset=0.0, z_offset=0.075):
    # changes workpiece orientation to match gripper frame orientation
    rotation_mat = np.array([
        [0, -1, 0, 0],
        [-1, 0, 0, 0],
        [0, 0, -1, 0],
        [0, 0, 0, 1]
    ])

    translation_vector = [x_offset, 0, z_offset]

    # x_displacement_mat = np.array([
        # [1, 0, 0, x_offset],
        # [0, 1, 0, 0],
        # [0, 0, 1, z_observation_clearance],
        # [0, 0, 0, 1]
    # ])

    # ros_pose = transform_pose(initial_pose, x_displacement_mat)
    ros_pose = translate_pose(initial_pose, translation_vector)
    ros_pose_rotated = transform_pose(ros_pose, rotation_mat)
    rpl_pose_rotated = Pose.from_ros_pose(ros_pose_rotated)
    return rpl_pose_rotated, ros_pose_rotated


def set_and_change_toolframe_from_mat(frame_name, T):
    ros_pose = matrix_to_pose(T)
    rpl_pose = Pose.from_ros_pose(ros_pose)
    set_tool_frame(frame_name, rpl_pose)
    change_tool_frame(frame_name)

def movel_ros(pose: ROSPose, velocity=0.1):
    rpl_pose = Pose.from_ros_pose(pose)
    movel(rpl_pose, velocity=velocity)

def movej_ros(pose: ROSPose, velocity_scale=0.1):
    rpl_pose = Pose.from_ros_pose(pose)
    movej(rpl_pose, velocity_scale=velocity_scale)

def save_poses_to_csv(poses, filename='radial_swipe_poses.csv'):
    """
    Save a list of geometry_msgs/Pose to a CSV file.

    :param poses: List of geometry_msgs/Pose
    :param filename: Name of the CSV file to save (default: 'radial_swipe_poses.csv')
    """
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        # Write header
        writer.writerow(['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])

        # Write poses
        for pose in poses:
            writer.writerow([
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w
            ])

def read_poses_from_csv(filename='radial_swipe_poses.csv'):
    """
    Read a list of geometry_msgs/Pose from a CSV file.

    :param filename: Name of the CSV file to read (default: 'radial_swipe_poses.csv')
    :return: List of geometry_msgs/Pose
    """
    poses = []
    with open(filename, 'r') as csvfile:
        reader = csv.reader(csvfile)
        next(reader)  # Skip header

        for row in reader:
            pose = ROSPose()
            pose.position.x = float(row[0])
            pose.position.y = float(row[1])
            pose.position.z = float(row[2])
            pose.orientation.x = float(row[3])
            pose.orientation.y = float(row[4])
            pose.orientation.z = float(row[5])
            pose.orientation.w = float(row[6])
            poses.append(pose)

    return poses
