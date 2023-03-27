'''
Example 10: Path blending
'''

from robot_command.rpl import *

set_units("mm", "deg")  # set units

def setup_test_user_and_tool_frame():
    # Creating a user frame just above the table to make sure the robot accidentally run in to it.
    set_user_frame("test_user_frame1", position=p[400, -200, 550, 0, 0, 0])
    change_user_frame("test_user_frame1")
    # Creating a tool frame to make sure the z-axis is oriented correctly
    set_tool_frame("test_tool_frame1", orientation=p[0, 0, 0, 180, 0, 0])
    change_tool_frame("test_tool_frame1")

# main() run in a loop
def main():
    setup_test_user_and_tool_frame()
    notify("This example will illustrate how to use the path blending function and also show the difference between a non blended and a blended move.\n\nFirst, the program will run a set of moves that are not blended.\n\nNext, the same set of moves will be ran again but they will be blended with a 0mm blend radius.\n\nAnd lastly, the same set of moves will be ran again but this time they will be blended with a blend radius of 30mm.", warning=True)
    notify("The up coming moves are not blended, this will help you see the difference between a non blended move and a blended move.", warning=True)
    
    movej(Pose())  # Go to home position
    
    # First without path blending
    movel(Pose())
    movel(Pose(x=100, y=100))
    movel(Pose(y=200))
    
    notify("This next round will run the same set of moves but will be blended with a 0mm blend radius.", warning=True)
    # Second, lets two moves with 0.0mm blend radius
    set_path_blending(True, 0)  # Enable path blending
    movel(Pose())
    movel(Pose(x=100, y=100))
    movel(Pose(y=200))
    sync()  # sync moves executed before this command
    set_path_blending(False)  # Disable path blending
    
    notify("This time the same set of moves will be blended with a blend radius of 30mm", warning=True)
    # Now lets do the same thing but with 30mm blend radius
    movel(Pose())
    set_path_blending(True, 30)  # Enable path blending
    movel(Pose(x=100, y=100))
    movel(Pose(y=200))
    sync()  # sync moves executed before this command
    set_path_blending(False)  # Disable path blending

    exit()
