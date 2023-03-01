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


def main():
    '''
    sleep() is a function that pauses the program for a given amount of time.
    sleep takes in seconds as an argument.
    '''
    draw_rect()
    notify("Wait 5 seconds")
    sleep(5)  # Sleep/wait 5 seconds
    draw_rect()  # Then draw rectangle
    
    notify("Wait 3 seconds")
    sleep(3)    # Sleep/wait 3 seconds
    draw_rect()  # Then draw rectangle
    exit()  # Exit main loop