from robot_command.rpl import *

def open_main_gripper():
    sleep(0.5)
    set_digital_out("lidar_gripper", True)
    sleep(0.5)
    pass

def close_main_gripper():
    sleep(0.5)
    set_digital_out("lidar_gripper", False)
    sleep(1.0)
    pass