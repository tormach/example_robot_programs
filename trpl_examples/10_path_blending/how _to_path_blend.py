'''
Example 10: Path blending
'''

from robot_command.rpl import *

set_units("mm", "deg")  # set units

'''
Since we need paths to blend, genPath() is our helper function for generating paths.
genPath() creates a path stretching along the y-axis. By default the path will have 100 points.
'''
def genPath(numPoints=100):
    if numPoints > 100:
        notify("Too many points in path")
        exit()
        # return []
    path = []
    for i in range(numPoints):
        path.append((0, i))
    return path

# main() run in a loop
def main():
    # Creating a user frame just above the table to make sure the robot accidentally run in to it.
    set_user_frame("user_frame1", position=p[400, -200, 550, 0, 0, 0])
    change_user_frame("user_frame1")
    # Creating a tool frame to make sure the z-axis is oriented correctly
    set_tool_frame("tool_frame1", orientation=p[0, 0, 0, 180, 0, 0])
    change_tool_frame("tool_frame1")

    path = genPath()  # Generate a path

    # Test with path blending
    movej(Pose())  # Go to home position
    set_path_blending(True, 0.0)  # Enable path blending, blend radius 0.0m
    for point in path:
        movel(Pose(x=point[0], y=point[1]))  # Move to point
    sync()  # sync moves executed before this command
    set_path_blending(False)  # Disable path blending

    # Test without path blending
    notify("Next let's try running the same path without path blending", warning=True)
    movej(Pose())  # Go to home position
    for point in path:
        movel(Pose(x=point[0], y=point[1]))  # Move to point

    exit()

