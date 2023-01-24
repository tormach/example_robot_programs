'''
Example 1.0: movel() and pose()/p[]
In this exmaple we are going to draw a triangle with the helpe of two functions Pose()/p[] and movel()
'''

from robot_command.rpl import *


# NOTE: always remeber to set your units
set_units("mm", "deg")

def main():
    waypoint_1 = p[0, 0, 0, 0, 0, 0]  # Creating pose waypoints using the pose factory short cut, p[] 
    waypoint_3 = Pose(150, 0, 0, 0, 0, 0) # We can also create pose waypoints using Pose().
    waypoint_2 = Pose(y = 100)  # Using Pose() we can set specific parametars, the rest will default to zero.
   
    # Notes: movel() will make the robot's end effector to move linearly between points.
    movej(waypoint_1)  # move to waypoint_1
    movel(waypoint_2, velocity=60, accel_scale = 1) # move to waypoint_2 with velocity at 60 and acceleration(0 - 1) at 1 or 100% 
    movel(waypoint_3, velocity=1000, duration=5)    # move to waypoint_3 with velocity at 1000 and within 5 sec
    movel(waypoint_1, velocity=1000, strict_limits=True)  # move to waypoint_3 with velocity at 1000 and within 5 sec
    
    exit() # After we are done we exit/stop the main() loop