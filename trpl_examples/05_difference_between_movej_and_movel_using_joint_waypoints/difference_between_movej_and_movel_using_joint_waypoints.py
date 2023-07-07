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
    notify("This example will be showing you the difference between movej() and movel().\nBefore each demonstration you will see a notification just like this letting you know what move will be demonstrated.", warning=True)
    
    # Joint moves
    notify("You will be seeing a joint move (movej())", warning=True)
    movej(j[-50, 0, 0, 0, 10, 0], velocity_scale=0.3)   # Rotate joint 1 to -50 degrees and joint 5 to 10 degrees at 30% velocity
    movej(j[50, 0, 0, 0, 10, 0], velocity_scale=0.3)  # Rotate joint 1 to  50 degrees and joint 5 to 10 degrees at 30% velocity
    movej(j[-50, 0, 0, 0, 10, 0], velocity_scale=0.3)  # Rotate joint 1 to  -50 degrees and joint 5 to 10 degrees at 30% velocity

    # Linear moves
    notify("You will be seeing a linear move (movel())", warning=True)
    movel(j[50, 0, 0, 0, 10, 0], velocity=300)  # Move to a waypoint with joint 1 rotated 50 degrees  and joint 5 rotated 10 degrees at velocity 300
    movel(j[-50, 0, 0, 0, 10, 0], velocity=300)   # Move to a waypoint with joint 1 rotated -50 degrees  and joint 5 rotated 10 degrees at velocity 300
    movel(j[50, 0, 0, 0, 10, 0], velocity=300)   # Move to a waypoint with joint 1 rotated 50 degrees  and joint 5 rotated 10 degrees at velocity 300

    exit()