

# Example 16: Digital input and output (Digital I/O)
# This example will be demonstrating how to get signals from input and output pins,
# by controlling a gripper using a proximity switch.
# You can find these output pins on one of the sides of the robot cabinet,
# or go to the status tab in the software UI and look at the Digital I/O section to the right.

# These pins make it easy to interact with external devices like buttons or grippers.



from robot_command.rpl import *


set_units("mm", "deg")  # set units

# Setting up global variables for the pins
GRIPPER_PIN = 7  	
PROX_SWITCH_PIN = 5

# Before we start opening and closing the gripper we need to set it in a known state.
set_digital_out(GRIPPER_PIN, False)  # Close gripper

#***************************************************************
# NOTE: For this program you will need a button and grippers 
#**************************************************************

notify("In this program, you will be opening and closing a gripper using a proximity switch.\n\nClick \"OK\" to continue if you have the gripper connected to output pin 7 and proximity switch conected to input pin 5 located on the robot cabinet.\n\nTo trigger the proximity switch put the proximity switch papendiculer to any metal surface.", warning=True)


# The main function runs in a loop
def main():
	input_pin_state = get_digital_in(PROX_SWITCH_PIN)  # Get the current state of the proximity switch.

	# The proximity switch state will be "False" when its in contact with a metal object.
	if input_pin_state is False:  # If we detect a metal object
		set_digital_out(GRIPPER_PIN, True)  # Open gripper
		sleep(2)  # Wait 2 seconds

	set_digital_out(GRIPPER_PIN, False)  # Close gripper
