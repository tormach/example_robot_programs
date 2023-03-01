'''
Example 7: Joints()/j[]
This example will be showing how Joints() differs from Pose()/p[]
If you haven't checkout pose_example.py in the current folder to see how Pose() differs from Joints()/j[]
'''
from robot_command.rpl import *

set_units("mm", "deg")  # set units

# A function to move in a rectangular formation
def draw_rect():
    movej(j[0,30,0,0,60,0])
    movel(j[-20,30,0,0,60,0])
    movel(j[-20,30,0,0,60,0])
    movel(j[-20, 10,30,0,55,0])
    movel(j[0, 10,30,0,55,0])
    movej(j[0,30,0,0,60,0])

#  main() runs in a loop
def main():
    '''
    Joints()/p[] waypoints are not affected by user frames and tool frames.
    The main difference from Pose() is that with Joints() you control specific joints to move the end effector  e.g Joints(j1=20, ...j6)/p[j1, ...j6]
    Below we create two user frames, one is horizontal and the other is vertical but if you run this script you will notice that even after changing to two different user frames the robot didn't change behavior.
    '''
    set_tool_frame("tool_frame1", orientation=p[0,0,0,180,0,0])  # Create a tool frame rotated 180 degrees on the x-axis (this will make sure the end effector movements are not flipped)
    change_tool_frame("tool_frame1")  # Use the created tool frame
    
    # Horizontal user frame
    set_user_frame("home_frame1", position=p[400,-200,550,0,0,0])  # Create first  user frame (this user frame is created just above Joint 2 of the robot)
    change_user_frame("home_frame1")  # Use the created user frame
    notify("Horizontal user frame is active")
    draw_rect()  # draw a rectangle
    
    # Vertical user frame
    set_user_frame("home_frame2", position=p[400,-200,550,0,0,0], orientation=p[0,0,0,90,0,0]) # Create second user frame
    change_user_frame("home_frame2")
    notify("Vertical user frame is active")
    draw_rect()  # draw a rectangle
    
    exit() # exit main loop