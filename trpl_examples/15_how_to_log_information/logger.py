
'''
Example 15: logging
This example program shows you how to log information to the status tab of the PathPilot UI.
'''
from robot_command.rpl import *
import rospy


def main():
    '''
    To log information, we will be using the rospy package that we imported.
    rospy has two functions that will be helping us with this task.
    -The rospy.logwarn() function is used to log warnings.
    -rospy.logerr() is used to log errors.
    It deosn't matter what logging function you decide to use but
    the key difference is that rospy.logwarn() is highlighted yellow in the status tab
    and rospy.logerr() is highlighted red.
    '''

    notify("This example program will be demonstrating how to log/print information to the status tab of the PathPilot UI.\nIf you check the status tab you will see that the program has logged values from 0-9.", warning=True)
    for i in range(10):  # For every value from 0 - 9
        # Logging a warning to the status tab
        rospy.logwarn("Warning log: "+str(i))
    # Logging an error to the status tab
    rospy.logerr("(End of program) Error log ")
    exit()
