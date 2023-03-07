# Example 7: difference_between_pose_and_joint_waypoints
The main differences between Joints() and Pose() waypoints:
* Pose() move the end effectir  

In the example we learn about:
* `movec()`: An arc/circular move command
* `movej()`: A joint move command. With this command, the robot end effector will move between points using the available efficient path, also considering angle limits for each joint.
* `Pose()`: This is used to create pose waypoints and its short form is p[].

### Learn more about the Tormach Robot Programming Language (TRPL):
https://tormach.atlassian.net/wiki/spaces/ROBO/pages/1930690719/Tormach+Robot+Programming+Language#TormachRobotProgrammingLanguage-robot_command.rpl.execute_trajectory
