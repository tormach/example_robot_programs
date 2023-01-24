'''
Example 1.3: movel() and pose()/p[]
In this example we will be drawing a number of triangles in a circle.
'''
# First we import necessery libraries for our program...
from robot_command.rpl import * # Needed to interface with the robot
import math # This will give us access to math functions like cos(), sin(), etc 

# Don't forget to set your units
set_units("mm", "deg")


def main():
    # We need to make sure we have the right user and tool frame
    
    # Below we create a tool frame to make sure it's oriented correctly. 
    # Since the tool frame's "z" axis is pointing down by defulat we need to rotate it to point up for what we are doing. 
    set_tool_frame("tool_frame1", orientation=p[0,0,0,180,0,0]) 
    change_tool_frame("tool_frame1") # After creating it we make sure we use it.
    
    # Make sure you set up a user frame, call it "user_frame1" or any name you wish
    # Learn how to set up a user frame here: https://www.youtube.com/watch?v=HHvmWXkA0xs
    change_user_frame("user_frame1")

    numOfTriangles = 50 # Number of triangles
    angleOffset = math.radians(360/numOfTriangles) # Angle offset 
    v_scl = 100    # Line scale
    curr_angle = 0 # Current angle 
    
    homeOffset_x = 100
    homeOffset_y = 100
    x = 0
    y = 0

    home_point = p[homeOffset_x, homeOffset_y, 0, 0, 0, 0]
    for i in range(numOfTriangles):
        '''
        NOTE: Below, you will notice that we used movej() instead of movel(). Yet
        the example was supposed to be talking about movel().
        The point is that it's good practice to run movej() as your first move command 
        for the very first time you run your program depending on what you are planning to do.
        The reason is that movel() is a linear command and movej() is a joint move commnad. 
        If the robot's joints were previously in a weird orientaion that locks its movements, a linear command might not work. 
        So joint move cammand (movej()) can be used to reorient the joints.
        '''
        # For each triangle we draw, ...
        movej(home_point) # Move to home point
        # Using the current angle calculate the 2nd point of the triangle...
        x = math.cos(curr_angle)*v_scl
        y = math.sin(curr_angle)*v_scl
        # Adjust the current (x, y) point to be relative to the home point by adding the home offset to it
        x = x + homeOffset_x
        y = y + homeOffset_y
        movel(p[x, y, 0, 0, 0, 0 ]) # Move to the 2nd point
   	    
        # Calculate the 3rd point of the triangle...
        temp_angle = curr_angle + math.radians(30) # Create a temporary angle  thats offset 30degs from the current angle 
        # Use the temporary angle to Calculate the 3rd point
        next_x = math.cos(temp_angle)*v_scl/2
        next_y = math.sin(temp_angle)*v_scl/2
        
        # Adjust the next (x, y) point to be relative to the home point by adding the home offset to it
        next_x = next_x + homeOffset_x
        next_y = next_y + homeOffset_y
        movel(p[next_x, next_y, 0, 0, 0, 0 ]) # Move to the 3rd point of the triangle
        # We could have included the homeOffset directly in the (next_x, next_y) calculation above like: next_x = math.cos(temp_angle)*v_scl/2 + homeOffset_x

   	    # Adjust the current angle by adding an offset (this will be used to draw the next triangle away from the previous one)
        curr_angle = curr_angle + angleOffset
        movel(home_point) # Move to a home point
    
    exit() # Exit/stop the main() loop when finished