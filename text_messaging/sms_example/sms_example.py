from robot_command.rpl import *
import sms

set_units("mm", "deg")
waypoint_1 = j[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
waypoint_2 = j[-7.210196351726281e-06, 17.94492052734985, 12.428081079119464, 8.654013076957924e-05, 59.62700291244165, -1.692654907354337e-05]

destination_phone_number = '+6085551212'
class Globals:
    def __init__(self):
        self.count = 0

g = Globals()

def main():
    movej(waypoint_1)
    movej(waypoint_2)
    g.count += 1
    sms.send_sms_alert('unused', destination_phone_number, 'This program completed ' + str(g.count) + " times.")
    if g.count == 3:
        exit()
