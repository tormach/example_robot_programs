'''
Example 7: movec() and Pose()/p[]
In this example we will be using movec() and Pose() to draw a circle.
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

# main runs in a loop
def main():
    '''
    As a new user you might want to run this program
    without worrying about setting up user frames or anything.
    So, below we create a test user frame that you can use
    right away without setting up anything.
    Learn how to set up a user frame here: https://www.youtube.com/watch?v=HHvmWXkA0xs
    '''
    set_user_frame("user_frame1", position=p[400, -200, 550, 0, 0, 0])  # Create user frame at position x = 400, y = -200 and z = 550
    change_user_frame("user_frame1")  # We use it
    '''
    Notify() is a function that throws popup messages on a screen.
    In this case we are using it to alert you about the created user
    frame and also render an image that illustrates
    where the user frame will be.
    '''
    notify("The image is just an illustration to show where the user frame we just created will be set. As you can see it will be set just above joint 2 (J2). If your table is clear click OK to continue.", warning=True, image_path="./tormach_user_frame_graphic.png")

    # The default tool frame has it's "z" axis pointing down, for what we are going to do we need it to be pointing up.
    # So we create a new tool frame with a name "tool_frame1" (you can give it any name you want) and
    # rotate it 180degs so that the "z" axis points up.
    set_tool_frame("tool_frame1", orientation=p[0, 0, 0, 180, 0, 0])
    change_tool_frame("tool_frame1")  # We make sure we are using it
    
    '''
    movec() moves the robot end effector in an arc based on two main points, the mid/interim point and the target point.
    '''
    # Setup xy variables
    x = 50
    y = 50

    start_waypoint = p[x, y, 0, 0, 0, 0]  # Create a start waypoint using the xy value above
    movej(start_waypoint)  # Move to the start_waypoint, we use movej() so that the robot arm re-orient in case it was in a restrictive pose

    # First arc

    # Calculate the interim point by offsetting it away and out of line between the current position and the target point
    x = x + 50 # Get current value of x, add 50 and store it back in x (a simplified version would be x += 50)
    y = y + 50 # Get current value of y, add 50 and store it back in y (a simplified version would be y += 50)
    interim_waypoint = p[x, y, 0, 0, 0, 0]  # Create interim point
    # Calculate the target point 
    x = x + 50 
    y = y - 50  
    target_waypoint = p[x, y, 0, 0, 0, 0]  # Create target point
    movec(interim_waypoint, target_waypoint, velocity=100, accel=10, duration=3)  # Draw arc

    # Second arc
    # Since we are creating a circle our next arc will be a mirrored version of the first arc, so same calculations but opposite operations

    # Calculate the interim point
    x = x - 50
    y = y - 50
    interim_waypoint = p[x, y, 0, 0, 0, 0]  # Create interim point
    '''
    Since our mirrored arc is cycling back to the starting point of the first arc and we already craeted "start_waypoint" 
    we can reuse start_waypoint as the target waypoint.
    '''
    movec(interim_waypoint, start_waypoint, velocity=50, accel=20, duration=5)  # Draw arc

    exit()  # exit main loop
