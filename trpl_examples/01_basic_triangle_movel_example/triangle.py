'''
Example 1.0: movel() and pose()/p[]
In this examples we are going to draw a triangle with the help of two functions Pose()/p[] and movel()

Pose()/p[], takes in  xyz position and ABC rotations as an argument, for example Pose(x=0,y=0,z=0,a=0, b=0,c=0)/p[x, y, z, a, b, c]. 
ABC: A(rotation on X-axis), B(rotation on Y-axis), C(rotation on Z-axis)

movel(), takes in: 
    * target - target waypoint
    * velocity - move velocity as absolute value, interpreted in terms of currently set machine units if quantity without units is given.
    * accel - move acceleration as absolute value, interpreted in terms of currently set machine units if quantity without units is given.
    * accel_scale -  move acceleration scaling factor 0.0 - 1.0
    * duration  - target move duration in seconds. If move duration based on other inputs is longer, the planned duration will be used.

    * strict_limits - (Will be discussed in future examples)
    * probe - (Will be discussed in future examples)
'''

from robot_command.rpl import *


# NOTE: always remember to set your units
set_units("mm", "deg")

# The main function runs in a loop
def main():
    '''
     As a new user you might want to run this program without worrying about setting up user frames or anything.
     So, below we create a test user frame that you can use right away without setting up anything.
    '''
    userFrame_pos_x = 400
    userFrame_pos_y = -200
    userFrame_pos_z = 550
    userFrame_rotation_A = 0 # rotation on x-axis
    userFrame_rotation_B = 0 # rotation on y-axis
    userFrame_rotation_C = 0 # rotation on z-axis
    set_user_frame("user_frame1", position=p[userFrame_pos_x, userFrame_pos_y, userFrame_pos_z, userFrame_rotation_A, userFrame_rotation_B, userFrame_rotation_C])  # Create user frame at position x = 400, y = -200 and z = 550 and A= 0, B = 0, C= 0
    change_user_frame("user_frame1")  # We use it
    '''
    Notify() is a function that throws popup messages on a screen.
    In this case we are using it to alert you about the created user
    frame and also render an image that illustrates where the user frame will be.
    '''
    notify("The image is just an illustration to show where the user frame we just created will be set. As you can see it will be set just above joint 2 (J2). If your table is clear click OK to continue.", warning =True, image_path ="./tormach_user_frame_graphic.png")

    # The default tool frame has it's "z" axis pointing down, for what we are going to do we need it to be pointing up.
    # So we create a new tool frame with a name "tool_frame1" (you can give it any name you want) and
    # rotate it 180degs so that the "z" axis points up.
    set_tool_frame("tool_frame1", orientation=p[0,0,0,180,0,0])
    change_tool_frame("tool_frame1")  # We make sure we are using it

    waypoint_1 = p[0, 0, 0, 0, 0, 0]  # Creating pose waypoints using the pose factory short cut, p[]
    waypoint_3 = Pose(150, 0, 0, 0, 0, 0)  # We can also create pose waypoints using Pose().
    waypoint_2 = Pose(y=100)  # Using Pose() we can set specific parameter's, the rest will default to zero.

    # Notes: movel() will make the robot's end effector to move linearly between points.
    movej(waypoint_1)  # move to waypoint_1
    movel(waypoint_2, velocity=60, accel_scale=1)  # move to waypoint_2 with velocity at 60 and acceleration(0 - 1) at 1 or 100% 
    movel(waypoint_3, velocity=1000, duration=5)    # move to waypoint_3 with velocity at 1000 and within 5 sec

    '''
    Notes: Below we set "strict_limits" to true, meaning any velocity command
    too high for the robot to physically achieve given the limitation of the
    maximum joint velocities will cause an error and the program to stop.
    Feel free to change "strict_limits" to "false" or change the velocity to
    something less than 1000 to see the effects.
    '''
    movel(waypoint_1, velocity=1000, strict_limits=True)  # move to waypoint_3 with velocity at 1000 and within 5 sec

    exit()  # After we are done we exit/stop the main() loop
