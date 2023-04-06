# Example 5: difference_between_movej_and_movel
In this example we show the difference between `movej()` and `movel()` using `Joints()/j[]` waypoints.
The video below demonstrates a joint move and a linear move.

https://user-images.githubusercontent.com/34427350/217036060-2243b740-f347-45e1-a520-37c5661323d6.mp4

In the example we learn about:

* `movej()`: A joint move command. With this command, the robot end effector will move between points using the available efficient path, also considering angle limits for each joint.
* `movel()`: A linear move command. With this command, a robot end effector will move between points linearly.
* `Joints()`: This is used to create joint waypoints and its short form is j[].
* `set_units()`: Set units that will be used in a program

### Learn more about the Tormach Robot Programming Language (TRPL):
https://tormach.atlassian.net/wiki/spaces/ROBO/pages/1930690719/Tormach+Robot+Programming+Language
