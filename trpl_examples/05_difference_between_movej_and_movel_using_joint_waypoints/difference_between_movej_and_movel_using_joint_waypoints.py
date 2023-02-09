'''
Example 5: movej(), movel() and Joints()/j[]
This example demonstrates the difference between a joint move and linear move
using Joints()/j[] waypoints.
'''

from robot_command.rpl import *


#  Set units
set_units("mm", "deg")

# The main function runs in a loop
def main():
    # Joint moves
    movej(j[-50, 0, 0, 0, 10, 0], velocity_scale=0.3)   # Rotate joint 1 to -50 degrees and joint 5 to 10 degrees using 30% velocity
    movej(j[50, 0, 0, 0, 10, 0], velocity_scale=0.3)  # Rotate joint 1 to  50 degrees and joint 5 to 10 degrees using 30% velocity
    movej(j[-50, 0, 0, 0, 10, 0], velocity_scale=0.3)  # Rotate joint 1 to  -50 degrees and joint 5 to 10 degrees using 30% velocity

    # Linear moves
    movel(j[50, 0, 0, 0, 10, 0], velocity=300)  # Move to a waypoint with joint 1 rotated 50 degrees  and joint 5 rotated 10 degrees at velocity 300
    movel(j[-50, 0, 0, 0, 10, 0], velocity=300)   # Move to a waypoint with joint 1 rotated -50 degrees  and joint 5 rotated 10 degrees at velocity 300
    movel(j[50, 0, 0, 0, 10, 0], velocity=300)   # Move to a waypoint with joint 1 rotated 50 degrees  and joint 5 rotated 10 degrees at velocity 300
