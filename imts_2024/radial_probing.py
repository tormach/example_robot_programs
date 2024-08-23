from robot_command.rpl import *
from math import pi
import csv
from datetime import datetime
import os

def create_csv_file(filename):
    if not os.path.exists(filename):
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['x', 'y', 'z'])

def append_to_csv(filename, x, y, z):
    with open(filename, 'a', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([x, y, z])

def radial_probing(angle_min: float, angle_max: float, angle_increment: float, r_min: float, r_max: float, r_increment: float, z_start: float, linear_speed: float):
    change_tool_frame("gripper_frame")
    set_units("m", "rad", "s")
    Y_TABLE_LIMIT = 0.500

    # Create CSV file with an ISO date format
    iso_date = datetime.now().strftime("%Y-%m-%dT%H-%M-%S")
    csv_filename = f"table_map_{iso_date}.csv"
    create_csv_file(csv_filename)

    movej(j[0.0, 0.0, 0.0, 0.0, pi/2, -pi/2])
    num_r_passes = int((r_max - r_min) // r_increment + 1)
    num_angle_passes = int((angle_max - angle_min) // angle_increment + 1)
    for i in range(num_r_passes):
        r = r_min + ( r_increment * i )
        movel(p[r, 0.0, z_start, pi, 0, 0])
        sleep(1.0)
        val = get_joint_values()
        old_joints = j[val.j1, val.j2, val.j3, val.j4, val.j5, val.j6]
        for a in range(num_angle_passes):
            new_joints = j[val.j1 + (a * angle_increment), val.j2, val.j3, val.j4, val.j5, val.j6]
            movej(new_joints, velocity_scale = 0.30)
            pose_up = get_pose()
            if pose_up.y > Y_TABLE_LIMIT:
                break
            pose_goal = pose_up.copy()
            pose_goal.z -= 0.10
            sleep(1)
            contact_pose = probel(pose_goal, v=linear_speed, v_retract=0.01, away=False, check_retract_contact=False)

            x = contact_pose.x
            y = contact_pose.y
            z = contact_pose.z

            # Append x,y,z to the CSV file
            append_to_csv(csv_filename, x, y, z)

        movej(old_joints)

def main():
    # radial_probing(angle_min=0,
                   # angle_max=pi/2,
                   # angle_increment=pi/8,
                   # r_min=0.3,
                   # r_max=0.8,
                   # r_increment=0.05,
                   # z_start=0.05,
                   # linear_speed=0.002)

    radial_probing(angle_min=0,
                   angle_max=pi/2,
                   angle_increment=pi/90,
                   r_min=0.3,
                   r_max=0.8,
                   r_increment=0.02,
                   z_start=0.05,
                   linear_speed=0.002)
    exit()
