# Example 7: drawing_a_circle_using_movec_and_pose_waypoints
This example shows how to draw a circle using `movec()` and `Pose()/p[]`

https://user-images.githubusercontent.com/34427350/219497162-86e8ec2f-d52e-4556-8459-387219d26522.mp4


In the example we learn about:
* `movec()`: An arc/circular move command
* `movej()`: A joint move command. With this command, the robot end effector will move between points using the available efficient path, also considering angle limits for each joint.
* `Pose()`: This is used to create pose waypoints and its short form is p[].

### Learn more about the Tormach Robot Programming Language (TRPL):
https://tormach.atlassian.net/wiki/spaces/ROBO/pages/1930690719/Tormach+Robot+Programming+Language#TormachRobotProgrammingLanguage-robot_command.rpl.execute_trajectory
