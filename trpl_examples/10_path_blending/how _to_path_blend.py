'''
Example 10: Path blending
'''

from robot_command.rpl import *

set_units("mm", "deg")  # set units


# main() run in a loop
def main():
    # Creating a user frame just above the table to make sure the robot accidentally run in to it.
    set_user_frame("user_frame1", position=p[400, -200, 550, 0, 0, 0])
    change_user_frame("user_frame1")
    # Creating a tool frame to make sure the z-axis is oriented correctly
    set_tool_frame("tool_frame1", orientation=p[0, 0, 0, 180, 0, 0])
    change_tool_frame("tool_frame1")

    movej(Pose())  # Go to home position

    # First lets two moves with 0.0mm blend radius
    set_path_blending(True, 0)  # Enable path blending
    movel(Pose(x=100, y=100))
    movel(Pose(y=100))
    sync()  # sync moves executed before this command
    set_path_blending(False)  # Disable path blending

    # Now lets do the same thing but with 30mm blend radius
    set_path_blending(True, 30)  # Enable path blending
    movel(Pose(x=100, y=100))
    movel(Pose(y=100))
    sync()  # sync moves executed before this command
    set_path_blending(False)  # Disable path blending

    exit()
