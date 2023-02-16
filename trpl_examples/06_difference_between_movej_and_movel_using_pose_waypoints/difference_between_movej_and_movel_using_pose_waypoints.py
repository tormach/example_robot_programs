'''
Example 6: movej(), movel() and Pose()/p[]
This example demonstrates the difference between a joint move and linear move
using Pose()/p[] waypoints.
'''

from robot_command.rpl import *

# Set your units
set_units("mm", "deg")

# The main function runs in a loop
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
    
    # Joint move with Pose()/p[]
    movej(p[0, 0, 0, 0, 0, 0],  velocity_scale=0.3)  # Move to cartesian point (x=0, y=0, z=0, a=0, b=0, c=0)
    movej(p[0, 300, 0, 0, 0, 0],  velocity_scale=0.3)  # Then move to y=300
    movej(p[0, 0, 0, 0, 0, 0],  velocity_scale=0.3)   # Then back to zero
    
    # linear move with Pose()/p[]
    movel(p[0, 0, 0, 0, 0, 0], velocity=300)  # Move cartesian point (x=0, y=0, z=0, a=0, b=0, c=0)
    movel(p[0, 300, 0, 0, 0, 0], velocity=300)  # Then move to y=300
    movel(p[0, 0, 0, 0, 0, 0], velocity=300)  # Then back to zero

    exit()  # Exit main loop

