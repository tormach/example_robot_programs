'''
Example 8: Pause example
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
    pause() is different form sleep() function in that it requires user input from the system UI to resume the program.
    Also you can make a pause optional by setting the "optional" parameter to true (pause(optional=True)).
    Optional pauses will be skipped unless you activate them by clicking "OPTIONAL PAUSE" button in the system UI.
    '''
    # Draw a rectangle two times
    draw_rect()
    draw_rect()
    notify("The program should be pause by the time you see this popup, click \"CYCLE START\" to resume")
    pause()  # Once the program reaches this point it will be paused till you click "CYCLE START" again to resume the program

    draw_rect()  # Draw a rectangle
    pause(optional=True)    # This pause will be skipped (activate it by clicking "OPTIONAL PAUSE" in the system UI)
    draw_rect()  # Draw a rectangle
    exit()  # Exit main loop
