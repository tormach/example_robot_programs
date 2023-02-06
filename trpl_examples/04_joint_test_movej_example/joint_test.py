'''
Example 4: movej() and Joints()/j[]
This example will show you how to use Joints() waypoints with
movej() by going through each joint, rotating it from 0 to 20 degrees and back to 0.
'''

from robot_command.rpl import *


# Set your units
set_units("mm", "deg")

# The main function runs in a loop
def main():
    angle = 20

    notify("Make sure the environment is clear", warning=True)  # Show a notification popup and set warning to true so that the program poses till "OK" is clicked
    movej(j[0, 0, 0, 0, 0, 0])  # Move all joints to angle zero

    # j1
    notify("Joint 1/ j1", warning=True)
    movej(j[angle, 0, 0, 0, 0, 0])  # Move joint 1 to given angle
    movej(j[0, 0, 0, 0, 0, 0])  # Move back to zero

    # j2
    notify("Joint 2/ j2", warning=True)
    movej(j[0, angle, 0, 0, 0, 0])  # Move joint 2 to given angle
    movej(j[0, 0, 0, 0, 0, 0])   # Move back to zero

    # j3
    notify("Joint 3/ j3", warning=True)
    # Using Joints() function you can specify which joint to move e.g Joints(j3=angle)
    movej(Joints(j3=angle))  # Move joint 3 to given angle
    movej(Joints())   # Move back to zero. Un empty Joints() will default all joint angles to zero

    # j4
    notify("Joint 4/ j4", warning=True)
    movej(Joints(j4=angle))  # Move joint 4 to given angle
    movej(Joints())   # Move back to zero

    # j5
    notify("Joint 5/ j5", warning=True)
    movej(Joints(j4=angle))  # Move joint 5 to given angle
    movej(Joints())   # Move back to zero

    # j6
    notify("Joint 6/ j6", warning=True)
    movej(Joints(j6=angle))  # Move joint 6 to given angle
    movej(Joints())   # Move back to zero

    exit()  # Exit/stop main() loop
