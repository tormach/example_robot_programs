'''
Example 7: Pose()/p[]
This example will be showing how Pose() differs from Joints()/j[]
After reading this continue to the joint_example.py in this folder to see how Joints() differs from Pose()/p[]
'''
from robot_command.rpl import *

set_units("mm", "deg")  # set units

# A function to move in a rectangular formation
def draw_rect(x=0, y=0, w=100, h=100):
    movej(p[x,y,0,0,0,0])
    movel(p[x+w,0,0,0,0,0])
    movel(p[x+w,y+h,0,0,0,0])
    movel(p[x,y+h,0,0,0,0])
    movel(p[x,y,0,0,0,0])

#  main() runs in a loop
def main():
    '''
    The main difference is that Pose() is affected by user frames and tool frames.
    And with Pose() you control the end effector in cartesian coordinates e.g p[x, y, z, a, b, c]/Pose(x=0, y=0, z=0, a=0, b=0, c=0)
    Below you will notice that we create two user frames, one is horizontal and the other is vertical.
    '''
    set_tool_frame("tool_frame1", orientation=p[0,0,0,180,0,0])  # Create a tool frame rotated 180 degrees on the x-axis (this will make sure the end effector movements are not flipped)
    change_tool_frame("tool_frame1")  # Use the created tool frame
 
    # Horizontal user frame
    set_user_frame("home_frame1", position=p[400,-200,550,0,0,0])  # Create first  user frame (this user frame is created just above Joint 2 of the robot)
    change_user_frame("home_frame1")  # Use the created user frame
    notify("Horizontal user frame is active")
    draw_rect()  # draw a rectangle

    # Vertical user frame
    set_user_frame("home_frame2", position=p[400,-200,550,0,0,0], orientation=p[0,0,0,90,0,0])  # Create second user frame
    change_user_frame("home_frame2")
    notify("Vertical user frame is active")
    draw_rect()  # draw a rectangle

    exit() # exit main loop
