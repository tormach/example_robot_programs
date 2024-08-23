from robot_command.rpl import *
from math import pi
import csv
from datetime import datetime
import os
import numpy as np
from math import pi
from geometry_msgs.msg import Pose as ROSPose
from geometry_utils import matrix_to_pose, rot_z, rpl_pose_to_matrix, set_and_change_toolframe_from_mat


def probe_surface_4_points(safe_pose, linear_speed):
    set_units("m", "rad", "s")
    results = []
    val = get_joint_values()
    old_joints = j[val.j1, val.j2, val.j3, val.j4, val.j5, val.j6]
    for angle in [-pi/2, 0, pi/2, pi]:
        new_joints = j[val.j1, val.j2, val.j3, val.j4, val.j5, val.j6 + angle]
        movej(new_joints, velocity_scale = 0.1)
        sleep(1)
        approach_pose = get_pose()
        approach_pose.z -= 0.1
        contact_pose = probel(approach_pose, v=linear_speed, v_retract=0.01, away=False, check_retract_contact=False)
        # notify(f"Contact pose {contact_pose.z}")
        xyz = (contact_pose.x, contact_pose.y, contact_pose.z)
        results.append(xyz)

    organized_results = [results[0], results[3], results[2], results[1]]
    return organized_results

def create_csv_file(filename):
    if not os.path.exists(filename):
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['X1', 'Y1', 'Z1', 'X2', 'Y2', 'Z2', 'X3', 'Y3', 'Z3', 'X4', 'Y4', 'Z4'])

def append_4_points_to_csv(organized_result, filename):
    with open(filename, 'a', newline='') as csvfile:
        writer = csv.writer(csvfile)
        flat_result = [coord for point in organized_result for coord in point]
        writer.writerow(flat_result)

def radial_probing_surface_4_points(angle_min: float, angle_max: float, angle_increment: float, r_min: float, r_max: float, r_increment: float, z_start: float, linear_speed: float):
    set_units("m", "rad", "s")
    Y_TABLE_LIMIT = 0.500 - 0.080

    # Create CSV file with an ISO date format
    iso_date = datetime.now().strftime("%Y-%m-%dT%H-%M-%S")
    csv_filename = f"table_4_points_{iso_date}.csv"
    create_csv_file(csv_filename)

    movej(j[0.0, 0.0, 0.0, 0.0, pi/2, -pi/2])
    num_r_passes = int((r_max - r_min) // r_increment + 1)
    num_angle_passes = int((angle_max - angle_min) // angle_increment + 1)

    for i in range(num_r_passes):
        change_tool_frame("custom_flange_frame")
        r = r_min + ( r_increment * i )
        movel(p[r, 0.0, z_start, pi, 0, 0])
        sleep(2.0)
        val = get_joint_values()
        old_joints = j[val.j1, val.j2, val.j3, val.j4, val.j5, val.j6]

        for a in range(num_angle_passes):
            new_joints = j[val.j1 + (a * angle_increment), val.j2, val.j3, val.j4, val.j5, val.j6 + (a * angle_increment)]
            change_tool_frame("probe_frame")
            movej(new_joints, velocity_scale = 0.30)
            sleep(1.0)
            pose_up = get_pose()
            if pose_up.y > Y_TABLE_LIMIT:
                break

            organized_result = probe_surface_4_points(pose_up, linear_speed)

            # Append 4 points to the CSV file
            append_4_points_to_csv(organized_result, csv_filename)

        movej(old_joints)

custom_flange_frame = np.array([
    [0, -1, 0, 0.0],
    [1, 0, 0, 0.0],
    [0, 0, 1, 0.0],
    [0, 0, 0, 1]
])

probe_frame = np.array([
    [0, -1, 0, -0.035],
    [1, 0, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])

set_units("m", "rad", "s")

set_and_change_toolframe_from_mat("custom_flange_frame", custom_flange_frame)
set_and_change_toolframe_from_mat("probe_frame", probe_frame)
change_user_frame("my_world_frame")

def main():
    radial_probing_surface_4_points(angle_min=0,
                   angle_max=pi/2,
                   angle_increment=pi/90,
                   r_min=0.3,
                   r_max=0.8,
                   r_increment=0.02,
                   z_start=0.220,
                   linear_speed=0.001)
    exit()
