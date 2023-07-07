'''
Example 14.1: Interrupts
This example will be illustrating how to use interrupts with external sensors like a button.
With Interrupts you can react to external input through the digital input output (DIO) pins, internal program triggers or ROS topic triggers.
'''

from robot_command.rpl import *

DIO_PIN = 4  # digital input output pin, where the button is connected
numberOfClicks = 0  # Global variable to store our click count

'''
The interrupt_handler(args) function is the custom function that we will be
passing down to the register_interrupt() function.
You might notice that the custom function has some parameter or argument "args",
that's because we expect register_interrupt() to return some values that are
then handed over to the custom function.
'''
def interrupt_handler(args):
	# This function will be called when an interrupt is triggered

	global numberOfClicks  # Referencing the global variable "numberOfClicks"
	numberOfClicks += 1  # Increment number of clicks by one
	notify("Number of clicks: "+str(numberOfClicks), warning=True)  # Show a popup with the updated click count

notify("This is a click counter example program that will show you how to use external hardware with the robot with the help of the register_interrupt() function.\n\nTo get started make sure you have connected a button or a similar hardware to DIO pin 4. Read the README file to learn more about DIO pins and figuring out which one you are connected to.\n\n Once everything is setup click \"OK\" to continue.Then trigger or click your button and a popup should show up telling you the number of clicks.")

def main():
	'''
	Below we register an interrupt by calling "register_interrupt()" function.
	This function expects a source of interrupt that will be used as a trigger.
	In our case we choose "DigitalInput" since we will be using an external hardware like a button.
	The register_interrupt() function also expects a digital input output (DIO) pin number. This number is the location where you connect your button or other external hardware.
	Lastly the register_interrupt() function expects a custom function (all th things you want to do when the interrupt is triggered goes in this function), in our case we called the function interrupt_handler(args).
	'''
	register_interrupt(InterruptSource.DigitalInput, DIO_PIN, interrupt_handler)  # NOTE:  An interrupt can be registered from outside or inside the main() function.
