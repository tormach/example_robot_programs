# Example 4: Joint_test_movej_example
This example demonstrates how to use movej() with Joints()/j[] waypoints.
The script will move each joint from j1 to j6 and display a Popup notifictaion telling you which joint will be moving next. 

In the example we learn about:

* `movej()`: A joint move command. With this command, the robot end effector will move between points using the available efficient path, also considering angle limits for each joint.
* `Joints()`: This is used to create joint waypoints and its short form is j[].
* `notify()`: Used to display popup messages on a screen
* `set_units()`: Set units that will be used in a program

### Learn more about the Tormach Robot Programming Language (TRPL):
https://tormach.atlassian.net/wiki/spaces/ROBO/pages/1930690719/Tormach+Robot+Programming+Language#TormachRobotProgrammingLanguage-robot_command.rpl.execute_trajectory
