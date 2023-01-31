# Example 1: basic_Triangle_movel_example
This is a basic example showing how to use `movel()` and `Pose()/p[]`. To make things easy a triangle is used as a path pattern.
If you are testing out the script just to see what it does it should work without configuring anything. Don't forget to clear the robot's environment to make sure it doesn't run into things. 

The video below shows what the script does, though when you run this script the robot arm will be moving at a safe distance above the table because of how the user frame is set up. To make the robot write on paper just like in the video you will need to properly set up a user frame. Learn how to set up a user frame from this YouTube video: https://www.youtube.com/watch?v=i_zQoZG7DYQ


https://user-images.githubusercontent.com/34427350/215850678-1b1f0470-a1b3-4e08-bdd9-f2684cb998eb.mp4

In the example we learn about:
* `movel()`: A linear move command. With this command, a robot end effector will move between points linearly.
* `Pose()`: This is used to create waypoints or target points and its short form is p[].
* `notify()`: Used to display popup messages on a screen
* `set_units()`: Set units that will be used in a program
* `set_user_frame()`: Used to create a new user frame 
* `change_user_frame()`: Used to set the current user frame that will be used in a program
* `movej()`: A joint move command. With this command, the robot end effector will move between points using the available efficient path,  also considering angle limits for each joint.


### Learn more about the Tormach Robot Programming Language:
https://tormach.atlassian.net/wiki/spaces/ROBO/pages/1930690719/Tormach+Robot+Programming+Language#TormachRobotProgrammingLanguage-robot_command.rpl.execute_trajectory
