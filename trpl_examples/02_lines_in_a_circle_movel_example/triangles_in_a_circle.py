'''
Example 1.2: movel() and pose()/p[]
In this example we will be drawing a number of lines in a circle.
'''
# First we import necessary libraries for our program...
from robot_command.rpl import *  # Needed to interface with the robot
import math  # This will give us access to math functions like cos(), sin(), etc 

# Setting our units is very important
set_units("mm", "deg")


def main():
    # First we need to make sure we have the right user and tool frame
    '''
     As a new user you might want to run this program
     without worrying about setting up user frames or anything.
     So, below we create a test user frame that you can
     use right away without setting up anything.
     Learn how to set up a user frame here: https://www.youtube.com/watch?v=HHvmWXkA0xs
    '''
    set_user_frame("user_frame1", position=p[400,-200,550,0,0,0])  # Create user frame at position x = 400, y = -200 and z = 550
    change_user_frame("user_frame1") # We use it
    '''
    Notify() is a function that throws popup messages on a screen. 
    In this case we are using it to alert you about the created user 
    frame and also render an image that illustrates where the user frame will be.
    '''
    notify("The image is just an illustration to show where the user frame we just created will be set. As you can see it will be set just above joint 2 (J2). If your table is clear click OK to continue.", warning=True, image_path="./tormach_user_frame_graphic.png")

    # The default tool frame has it's "z" axis pointing down, for what we are going to do we need it to be pointing up.
    # So we create a new tool frame with a name "tool_frame1" (you can give it any name you want) and
    # rotate it 180degs so that the "z" axis points up.
    set_tool_frame("tool_frame1", orientation=p[0,0,0,180,0,0])
    change_tool_frame("tool_frame1")  # We make sure we are using it

    numOflines = 100  # Number of lines
    a_offs = math.radians(360/numOflines)  # Angle offset
    v_scl = 100     # Line scale
    curr_ang = 0    # Current angle

    # Home offsets can be used to reposition the drawing
    homeOffset_x = 100
    homeOffset_y = 100

    # Current (x,y)
    x = 0
    y = 0

    safeHight = 100
    home_point = p[homeOffset_x, homeOffset_y, 0, 0, 0, 0]

    # For each line we draw...
    for i in range(numOflines):
        # For each line we draw...
        movej(home_point)  # Move to the home point to get ready

        # Calculate the next point to draw a line
        x = math.cos(curr_ang)*v_scl
        y = math.sin(curr_ang)*v_scl
        # Adjust the current (x, y) point to be relative to the home point by adding the home offset to it
        x = x + homeOffset_x
        y = y + homeOffset_y

        movel(p[x, y, 0, 0, 0, 0])  # Move to the next point to draw a line
        movel(p[x, y, safeHight, 0, 0, 0])  # Move up to a safe height

        # Update current angle by adding an angle offset
        curr_ang = curr_ang + a_offs

    exit()  # Exit/stop the main() loop when finished
