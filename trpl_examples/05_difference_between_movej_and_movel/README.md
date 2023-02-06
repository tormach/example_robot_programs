# Example 5: difference_between_movej_and_movel
In this example we will see the diffrence between `movej()` and `movel()` using `Joints()/j[]` waypoints.
So far we know that  `movej()` is a joint move and `movel()` is a linear move, in the video below we see what it means.

https://user-images.githubusercontent.com/34427350/217036060-2243b740-f347-45e1-a520-37c5661323d6.mp4

In the example we learn about:

* `movej()`: A joint move command. With this command, the robot end effector will move between points using the available efficient path, also considering angle limits for each joint.
* `Joints()`: This is used to create joint waypoints and its short form is j[].
* `notify()`: Used to display popup messages on a screen
* `set_units()`: Set units that will be used in a program

### Learn more about the Tormach Robot Programming Language (TRPL):
https://tormach.atlassian.net/wiki/spaces/ROBO/pages/1930690719/Tormach+Robot+Programming+Language#TormachRobotProgrammingLanguage-robot_command.rpl.execute_trajectory
