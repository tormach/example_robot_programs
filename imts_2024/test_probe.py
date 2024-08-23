from robot_command.rpl import *
import numpy as np
from math import pi

from geometry_msgs.msg import Pose as ROSPose
from geometry_utils import matrix_to_pose, rot_z, rpl_pose_to_matrix, set_and_change_toolframe_from_mat

change_user_frame("my_world_frame")
set_and_change_toolframe_from_mat("flange_frame", np.eye(4))

def probe_surface_now():
    set_units("m", "rad", "s")
    current_pose = get_pose()
    h_matrix = rpl_pose_to_matrix(current_pose)
    tilt_1_rad, tilt_2_rad = probe_surface(h_matrix)
    tilt_1_deg = tilt_1_rad * 180 / pi
    tilt_2_deg = tilt_2_rad * 180 / pi
    notify(f"Tilt 1: {tilt_1_deg:.2f}°, Tilt 2: {tilt_2_deg:.2f}°", warning=True)


def probe_surface(safe_pose):
    set_units("m", "rad", "s")
    results = []
    for angle in [-pi/2, 0, pi/2, pi]:
        safe_pose_rotated = safe_pose @ rot_z(angle)
        ros_pose = matrix_to_pose(safe_pose_rotated)
        rpl_pose = Pose.from_ros_pose(ros_pose)
        movel(rpl_pose, velocity = 0.1)
        sleep(1)
        approach_pose = rpl_pose.copy()
        approach_pose.z -= 0.1
        contact_pose = probel(approach_pose, v=0.01, v_retract=0.1, away=False, check_retract_contact=False)
        # notify(f"Contact pose {contact_pose.z}")
        results.append(contact_pose.z)

    base_diameter = 0.07

    tilt_1_rad = np.arctan((results[2] - results[0]) / base_diameter)
    tilt_2_rad = np.arctan((results[1] - results[3]) / base_diameter)

    return tilt_1_rad, tilt_2_rad


def main():
    set_units("m", "rad", "s")

    safe_pose = np.array([
        [1, 0, 0, 0.6],
        [0, -1, 0, 0.0],
        [0, 0, -1, 0.3],
        [0, 0, 0, 1]
    ])

    movej(j[0, 0.8111, 0.2336, 0, 0.5262, 0])

    # current_pose = get_pose()

    probe_surface(safe_pose)

    exit()
