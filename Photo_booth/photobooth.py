set_units("mm", "deg")
Face_center = p[424.64229464530945, 113.15371096134186, 714.5848870277405, -14.491903672233091, -41.96216022431763, -152.24350992497426]
low_left = p[506.7787766456604, -390.88907837867737, 430.9060275554657, -15.336350234168076, -43.20176755350519, -138.5756449953985]
lowmid = p[573.8926529884338, 58.7795227766037, 546.1881160736084, -14.5484106206423, -43.517000671366475, -156.36089236612438]
mid_right = p[419.9662506580353, 411.09591722488403, 721.4654684066772, -25.594926337414734, -57.82035431309877, -160.6528233936998]
high_right = p[270.4237997531891, 399.63236451148987, 1038.7722253799438, -27.321706069198974, -58.70081199529569, -154.13884608280426]
low_right = p[455.8054804801941, 475.51119327545166, 416.11889004707336, -16.247987268383685, -41.22605797904771, -167.64327443558602]
high_left = p[504.0569305419922, -393.21833848953247, 769.1679000854492, -20.619444377210264, -48.72997961668801, -132.2241039382612]

import sh
import os
from random import randint

try:
    sh.dpkg_query("-W", "mpg321")
except:
    sh.sudo("apt-get", "install", "-y", "mpg321")

last_pos = -1

def rand_sound():
    int_sound = randint(0, 10)
    sound_num = str(int_sound)
    return sound_num


def speak(sound_num):
    filename = sound_num + ".mp3"
    filename = filename.replace(" ", "_")
    print(filename)
    if os.path.exists(filename):
        os.system("mpg321 " + filename)


def rand_pos():
    global last_pos
    pos_num = last_pos
    while pos_num == last_pos:
        pos_num = randint(0, 5)
    last_pos = pos_num
    return pos_num

def repos_robot(pos_num):
    if pos_num == 1:
        movej(Face_center)
    if pos_num == 2:
        movej(high_right)
    if pos_num == 3:
        movej(low_left)
    if pos_num == 4:
        movej(high_left)
    if pos_num == 5:
        movej(lowmid)
    if pos_num == 0:
        movej(mid_right)

def main():
    while get_digital_in("button") is True:
        sync()
    pos_num = rand_pos()
    repos_robot(pos_num)
    set_digital_out(3, True)
    sleep(0.5)
    sound_num = rand_sound()
    set_digital_out(3, False)
    speak(sound_num)
    sleep(1.0)
