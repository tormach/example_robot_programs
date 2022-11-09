from robot_command.rpl import *
from machinekit import hal


def main():
    # use HALMETER on the settings screen for a convenient way to see which values are available
    pin = hal.Pin('lcec.0.1.torque-actual-value')
    value = pin.get()
    notify("Torque on J2 is " + str(value))