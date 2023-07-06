'''
Example 4: movej() and Joints()/j[]
This example will show you how to use Joints() waypoints with
movej() by going through each joint, rotating it from 0 to 20 degrees and back to 0.
'''

from robot_command.rpl import *

ANGLE = 20
# Set your units
set_units("mm", "deg")

# The main function runs in a loop
def main():
    notify("Make sure the environment is clear, the robot will be rotating all joints to a zero.", warning=True)  # Show a notification popup and set warning to true so that the program poses till "OK" is clicked
    movej(j[0, 0, 0, 0, 0, 0], velocity_scale=0.2)  # Move all joints to angle zero at 20% velocity

    # j1
    notify("Moving joint 1 (j1)", warning=True)
    movej(j[ANGLE, 0, 0, 0, 0, 0], velocity_scale=0.2)  # Move joint 1 to given angle at 20% velocity
    movej(j[0, 0, 0, 0, 0, 0], velocity_scale=0.2)  # Move back to zero at 20% velocity

    # j2
    notify("Moving joint 2 (j2)", warning=True)
    movej(j[0, ANGLE, 0, 0, 0, 0], velocity_scale=0.1)  # Move joint 2 to given angle at 10% velocity
    movej(j[0, 0, 0, 0, 0, 0], velocity_scale=0.1)   # Move back to zero at 10% velocity

    # j3
    notify("Moving Joint 3  (j3)", warning=True)
    # Using Joints() function you can specify which joint to move e.g Joints(j3 = angle)
    movej(Joints(j3=ANGLE), velocity_scale=0.2)  # Move joint 3 to given angle at 20% velocity
    movej(Joints(), velocity_scale=0.2)   # Move back to zero. Un empty Joints() will default all joint angles to zero at 20% velocity

    # j4
    notify("Moving joint 4 (j4)", warning=True)
    movej(Joints(j4=ANGLE), velocity_scale=0.2)  # Move joint 4 to given angle at 20% velocity
    movej(Joints(), velocity_scale=0.2)   # Move back to zero at 20% velocity

    # j5
    notify("Moving joint 5 (j5)", warning=True)
    movej(Joints(j5=ANGLE), velocity_scale=0.2)  # Move joint 5 to given angle at 20% velocity
    movej(Joints(), velocity_scale=0.2)   # Move back to zero at 20% velocity

    # j6
    notify("Moving joint 6 (j6)", warning=True)
    movej(Joints(j6=ANGLE), velocity_scale=0.2)  # Move joint 6 to given angle at 20% velocity
    movej(Joints(), velocity_scale=0.2)   # Move back to zero at 20% velocity

    exit()  # Exit/stop main() loop
