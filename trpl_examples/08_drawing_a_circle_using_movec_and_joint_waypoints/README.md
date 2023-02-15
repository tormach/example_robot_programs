# Example 8: drawing_a_circle_using_movec_and_joint_waypoints
This example show how to move the robot end effector in a circle using `movec()` and `Joints()/j[]`

[NOTE: Add video]

In the example we learn about:
* `movec()`: An arc/circular move command
* `movej()`: A joint move command. With this command, the robot end effector will move between points using the available efficient path, also considering angle limits for each joint.
* `Joints()`: This is used to create joint waypoints and its short form is j[].

### Learn more about the Tormach Robot Programming Language (TRPL):
https://tormach.atlassian.net/wiki/spaces/ROBO/pages/1930690719/Tormach+Robot+Programming+Language#TormachRobotProgrammingLanguage-robot_command.rpl.execute_trajectory
