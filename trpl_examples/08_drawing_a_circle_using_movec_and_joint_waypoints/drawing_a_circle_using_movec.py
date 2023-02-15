'''
Example 8: movec() and Joints()/j[]
In this example we will be using movec() and Joint() to move the robot end effector in a circle.
movec() has the following arguments or parameters movec(interim, target, probe, velocity, accel, accel_scale, duration, strict_limits) :
-interim: interim/mid waypoint
-target: target waypoint
-probe (We won't be looking at this one in this example)
-velocity: Move velocity as absolute value, interpreted in terms of currently set machine units if quantity without units is given.
-accel: Move acceleration as absolute value, interpreted in terms of currently set machine units if quantity without units is given.
-accel_scale: Move acceleration scaling factor 0.0 - 1.0
-duration: Target move duration in seconds. If move duration based on other inputs is longer, the planned duration will be used.
-strict_limits: Enforces strict limits. Moves violating the velocity and acceleration limits will error.
'''

from robot_command.rpl import *

# Set your units
set_units("mm", "deg")

def main():
    '''
    movec() moves the robot end effector in an arc based on two main points, the mid/interim point and the target point.
    '''
    
    # To create the starting point we rotate joint 1 to point to the starting waypoint (in our case -20 degrees). 
    # Also slightly tilt joint 5 so that the robot is able to calculate the move.
    start_waypoint = j[-20, 0, 0, 0, 4, 0]  # Create a start waypoint
    movej(start_waypoint)   # Move to the start_waypoint, we use movej() so that the robot arm re-orients in case it was in a restrictive pose
    
    # First arc
    interim_waypoint = j[0, -20, 0, 0, 4, 0]  # Create interim/mid waypoint
    target_waypoint = j[20, 0, 0, 0, 4, 0]  # Create target waypoint
    movec(interim_waypoint, target_waypoint, velocity=100, accel=10, duration=3)  # Perform arc movement
    
    # Seconde arc
    # Since we are creating a circle our next arc will be a mirrored version of the first arc, so we will use opposite values of the first arc
    start_waypoint = j[20, 0, 0, 0, -4, 0]
    movej(start_waypoint)  # We use movej here to move to the starting point but also re-orient the arm to be able to perform the next arc movement
    interim_waypoint = j[0, 20, 0, 0, -4, 0]  # Create interim/mid waypoint
    target_waypoint = j[-20, 0, 0, 0, -4, 0]  # Create target waypoint
    movec(interim_waypoint, target_waypoint, velocity=60, accel=10, duration=4)  # Perform arc movement
