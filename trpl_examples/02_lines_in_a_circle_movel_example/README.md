# Example 2: lines_in_a_circle_movel_example
In this example, we will be drawing a number of triangles in a circle. 
In the video below you will see what the script does, but if you download and run this script the robot will perform the movements at a safe distance above your table. 
So make sure the table is clear.To make the robot write on paper just like in the video you will need to properly set up a user frame. Learn how to set up a user frame from this youtube video: https://www.youtube.com/watch?v=i_zQoZG7DYQ

https://user-images.githubusercontent.com/34427350/215869991-a1a22497-edc6-402c-968f-09f947878fe7.mp4

In the example we learn about:

* `movel()`: A linear move command. With this command, a robot end effector will move between points linearly.
* `Pose()`: This is used to create waypoints or target points and its short form is p[].
* `notify()`: Used to display popup messages on a screen
* `set_units()`: Set units that will be used in a program
* `set_user_frame()`: Used to create a new user frame
* `change_user_frame()`: Used to set the current user frame that will be used in a program
* `movej()`: A joint move command. With this command, the robot end effector will move between points using the available efficient path, also considering angle limits for each joint.

### Learn more about the Tormach Robot Programming Language:
https://tormach.atlassian.net/wiki/spaces/ROBO/pages/1930690719/Tormach+Robot+Programming+Language#TormachRobotProgrammingLanguage-robot_command.rpl.execute_trajectory
