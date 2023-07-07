'''
Example 14.2: Interrupts
This example will be illustrating how to trigger interrupts within the program.
With Interrupts you can react to external input through the digital input output (DIO) pins, internal program triggers or ROS topic triggers.
'''

from robot_command.rpl import *

DIO_PIN = 4  # digital input output pin (since we are not triggering the interrupt with any external hardware we can use any of the DIO pins available)
numberOfInterrupts = 0

'''
The interrupt_handler(args) function is the custom function that we will be
passing down to the register_interrupt() function.
You might notice that the custom function has some parameter or argument "value",
that's because we expect register_interrupt() to return some values that are
then handed over to the custom function.
'''
def interrupt_handler(value):
	if value:
		notify("Number of interrupts: "+str(value)+"\nClick \"OK\" to continue", warning=True)


'''
Below we register an interrupt by calling "register_interrupt()" function. 
This function expects a source of interrupt that will be used as a trigger.
In our case we choose "Program" since we will be triggering interrupts from within the program.
The register_interrupt() function also expects a digital input output (DIO) pin number.
This number is the location where you connect your button or other external hardware,
but for this program we won't be using any external hardware so that number will be used as a "communication channel ID".
Lastly the register_interrupt() function expects a custom function (all th things you want to do when the interrupt is triggered goes in this function), in our case we called the function interrupt_handler(args).

'''
register_interrupt(InterruptSource.Program, DIO_PIN, interrupt_handler)  # NOTE:  An interrupt can be registered from outside or inside the main() function.

notify("This is an interrupt counter example program that will show you how to trigger interrupts within the program width the help of trigger_interrupt() function.\n Click ok to continue.", warning=True)

def main():
	global numberOfInterrupts
	numberOfInterrupts += 1  # increment number of interrupts by 1
	
	'''
	To trigger interrupts within the program we use trigger_interrupt() function.
	This function takes in two parameter or arguments. The first one is the DIO_PIN/name "communication channel ID" (it should be the same us the one registered interrupt DIO pin).
	The second one is the value or message you want to send down to the interrupt (this will then be relayed back to your custom function in our case its "interrupt_handler(value)")
	'''
	trigger_interrupt(DIO_PIN, numberOfInterrupts)
