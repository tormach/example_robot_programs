'''
Example 8: Sleep example 
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

def setup_test_user_and_tool_frame():
    # Creating a user frame just above the table to make sure the robot accidentally run in to it.
    set_user_frame("test_user_frame1", position=p[400, -200, 550, 0, 0, 0])
    change_user_frame("test_user_frame1")
    # Creating a tool frame to make sure the z-axis is oriented correctly
    set_tool_frame("test_tool_frame1", orientation=p[0, 0, 0, 180, 0, 0])
    change_tool_frame("test_tool_frame1")

def main():
    setup_test_user_and_tool_frame()
    '''
    sleep() is a function that pauses the program for a given amount of time.
    sleep takes in seconds as an argument.
    '''
    notify("This program will be demonstrating how the sleep() function works.\n First, the robot will draw a rectangle and then pause for 5 seconds.\nThen it will draw another rectangle and pause for 2.5 seconds.\nLastly, it will draw another rectangle.", warning=True)
    draw_rect()  # Draw triangle
    
    sleep(5)  # Sleep/wait 5 seconds
    draw_rect()  # Then draw rectangle

    sleep(2.5)    # Sleep/wait 2.5 seconds
    draw_rect()  # Then draw rectangle
    exit()  # Exit main loop
