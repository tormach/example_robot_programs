# Example 7: difference_between_pose_and_joint_waypoints
The main differences between Joints() and Pose() waypoints:
* Pose() moves the end effector in xyz Cartesian coordinates e.g Pose(x=0, y=0, z=0, a=0, b=0, z=0)
* Pose() is affected by user frames
While:
* Joints() moves the end effector by specifying angles for each motor joint e.g Joints(j1=0,...,j6=0)
* Joints() is not affected by user frames

### Learn more about the Tormach Robot Programming Language (TRPL):
https://tormach.atlassian.net/wiki/spaces/ROBO/pages/1930690719/Tormach+Robot+Programming+Language
