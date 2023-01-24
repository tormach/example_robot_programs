'''
Example 1.2: movel() and pose()/p[]
In this example we will be drawing a number of lines in a circle.
'''
# First we import necessery libraries for our program...
from robot_command.rpl import * # Needed to interface with the robot
import math # This will give us access to math functions like cos(), sin(), etc 

# Setting our units is very important
set_units("mm", "deg")


def main():
    # First we need to make sure we have the right user and tool frame
    
    # Below we create a tool frame to make sure it's oriented correctly. 
    # Since the tool frame's "z" axis is pointing down by defulat we need to rotate it to point up for what we are going to be doing. 
    set_tool_frame("tool_frame1", orientation=p[0,0,0,180,0,0]) 
    change_tool_frame("tool_frame1") # After creating it, we are use it.
    
    # Make sure you set up a user frame, call it "user_frame1" or any name you wish
    # Learn how to set up a user frame here: https://www.youtube.com/watch?v=HHvmWXkA0xs
    change_user_frame("user_frame1")

    numOflines = 100  # Number of lines
    a_offs = math.radians(360/numOflines) # Angle offset
    v_scl = 100     # Line scale
    curr_ang = 0    # Current angle
    
    # Home offsets can be used to roposition the drawing
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
        movej(home_point) # Move to the home point to get ready
        
        # Calculate the next point to draw a line
        x = math.cos(curr_ang)*v_scl
        y = math.sin(curr_ang)*v_scl
        # Adjust the current (x, y) point to be relative to the home point by adding the home offset to it
        x = x + homeOffset_x
        y = y + homeOffset_y
        
        movel(p[x, y, 0, 0, 0, 0 ]) # Move to the next point to draw a line
        movel(p[x, y, safeHight, 0, 0, 0]) # Move up to a safe height
        
        # Update current angle by adding an angle offset
        curr_ang = curr_ang + a_offs
    
    exit() # Exit/stop the main() loop when finished