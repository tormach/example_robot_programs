# Example 14: Interrupt
This example illustrates how to register and trigger interrupts.
Interrupts are useful when trying to get input from external sensors like buttons.
In this project folder you will find two example programs `digital_input_interrupt.py` and `program_interrupt.py`.

The `digital_input_interrupt.py` example program focuses on `InterruptSource.DigitalInput` as the interrupt source.
To get this program running you will need an external sensor like a button connected to Digital input output (DIO) pin 4.
To find the number of the DIO pin you connect to, look at the number next to the pin on the cabinet or just go to the status tab in the software and look at the highlighted pin number (just like in the image below).

![DIO pin image](/trpl_examples/14_interrupts/assets/dio_pins_indecator.png)


The `program_interrupt.py` example program focuses on `InterruptSource.Program` as the interrupt source.


### Main functions used in these example programs
`register_interrupt(source, nr_or_name, custom_function)`: Registers a interrupt function to an interrupt source.
* `source` - Interrupt source type (e.g. `InterruptSource.DigitalInput`, `InterruptSource.Program`, `InterruptSource.UserIo`)
* `nr_or_name` - Number or name of the interrupt source, e.g. 1 for Digital Input 1.
* `custom_function` -  The function which should be called when the interrupt is triggered, if None is passed, this unregisters and disables the interrupt.

`trigger_interrupt(nr, value)`: Triggers a program interrupt.
* `nr` -  Program interrupt number which should be triggered (this should be the same as the `nr_or_name` that you pass to the `register_interrupt()` function).
* `value` – Value which is passed along with the triggered interrupt.


### Learn more about the Tormach Robot Programming Language (TRPL):
https://tormach.atlassian.net/wiki/spaces/ROBO/pages/1930690719/Tormach+Robot+Programming+Language