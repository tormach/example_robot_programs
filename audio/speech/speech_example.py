from robot_command.rpl import *
from robot_speech import speak
set_units("mm", "deg")

home_position = j[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
top = j[2.3633695101540545e-05, 16.300770176735423, -32.88067734206064, -6.790670273396017e-05, 106.58001689294645, 0.00010666918179155637]

def main():
    movej(home_position)
    movej(top)
    speak("hello")