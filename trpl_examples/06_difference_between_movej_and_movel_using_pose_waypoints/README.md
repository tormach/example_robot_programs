# Example 6: difference_between_movej_and_movel
In this example we show the difference between `movej()` and `movel()` using `Pose()/p[]` waypoints.
The video below demonstrates a joint move and a linear move.

https://user-images.githubusercontent.com/34427350/219496929-47c0844f-0214-4a06-a296-09cc6f5ad004.mp4


In the example we learn about:

* `movej()`: A joint move command. With this command, the robot end effector will move between points using the available efficient path, also considering angle limits for each joint.
* `movel()`: A linear move command. With this command, a robot end effector will move between points linearly.
* `Pose()`: This is used to create pose waypoints and its short form is p[].
* `set_units()`: Set units that will be used in a program

### Learn more about the Tormach Robot Programming Language (TRPL):
https://tormach.atlassian.net/wiki/spaces/ROBO/pages/1930690719/Tormach+Robot+Programming+Language

