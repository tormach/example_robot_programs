'''
Example 13: Using get_param() and set_param()
This program will demonstrate how to store and manage data using get_/set_param().
Data stored using set_param() will be persistent, meaning you can still access the data even after stopping and starting the program. 
Also the data can be accessed between programs. 

To store data we call set_param() which takes in two arguments.
set_param(name, value):
    name: a string that will be used as an access key for the data you store.
    value: The data you want to store. It can be any python or RPL data type

To get the stored data we call get_param() which also takes in two arguments but the second argument is optional.
get_param(name, default):
    name: A string access key that the data was store under.
    default: The default value that will be returned if nothing was found. By default "None" is returned as the default value.

To delete the saved data, we use delete_param() which takes in one argument.
delete_param(name):
    name: The string access key that the data was saved under.
'''

from robot_command.rpl import *

set_units("mm", "deg")

def setup_test_user_and_tool_frame():
    # Creating a user frame just above the table to make sure the robot accidentally run in to it.
    set_user_frame("test_user_frame1", position=p[400, -200, 550, 0, 0, 0])
    change_user_frame("test_user_frame1")
    # Creating a tool frame to make sure the z-axis is oriented correctly
    set_tool_frame("test_tool_frame1", orientation=p[0, 0, 0, 180, 0, 0])
    change_tool_frame("test_tool_frame1")


def main():
    setup_test_user_and_tool_frame()

    notify("This example will be showing you how to use get_/set_param().\nAt the beginning of the program we will try getting the saved waypoints, if there is any.\nIf nothing was found we will create new ones and save them.", warning=True)
    # Getting data that was stored, if nothing was stored, "None" will be returned by default.
    waypoint_1 = get_param("wp_1")
    waypoint_2 = get_param("wp_2")
    waypoint_3 = get_param("wp_3")

    # Check if retrieved data exists or is empty
    if (waypoint_1 == None and waypoint_2 == None and waypoint_3 == None): # If nothing was saved ...
        # Create and save waypoints, using there corresponding names as keys which we will use to access them.
        set_param("wp_1", Pose())
        set_param("wp_2",  Pose(x=100))
        set_param("wp_3",  Pose(y=100))

        # Getting the saved waypoints
        waypoint_1 = get_param("wp_1")
        waypoint_2 = get_param("wp_2")
        waypoint_3 = get_param("wp_3")

        notify("No waypoints were found, so new ones have been created and saved.\nClick \"OK\" to draw them", warning=True)
    else:
        notify("It looks like there were  some saved waypoints.\n\nThe saved waypoints have been retrieved and are ready to be ran:\n\nwaypoint_1:\n"+str(waypoint_1)+"\n\nwaypoint_2:\n"+str(waypoint_2)+"\n\nwaypoint_3:\n"+str(waypoint_3) +"\n\n Click \"OK\" to draw them", warning=True)

    # Draw waypoints
    movej(waypoint_1)
    movel(waypoint_2)
    movel(waypoint_3)
    movel(waypoint_1)

    # Handling data deletion ...
    delete_stored_data = input("The stored data will be persistent, meaning you will still be able to access it even after you stop/abort the program. You can delete that data by calling the delete_param() function.\n\n If you want to delete the saved waypoints enter \"yes\".",default="no" )
    if (delete_stored_data == "yes" or delete_stored_data == "Yes"):
        # Delete saved data
        delete_param("wp_1")
        delete_param("wp_2")
        delete_param("wp_3")
