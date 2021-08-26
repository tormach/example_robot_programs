from robot_command.rpl import *
set_units("mm", "deg")
waypoint_1 = j[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
waypoint_2 = j[28.853091059148877, 26.881198981238494, -0.515076904343126, 4.969616689786745e-17, 63.63424841176544, 28.852968282132252]


from mms import take_and_send_image

def main():
    movej(waypoint_1)
    movej(waypoint_2)
    take_and_send_image()
    exit()
