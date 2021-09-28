from robot_command.rpl import *

import csv

set_units("mm", "deg")
filename = "probe_file.csv"
origin = p[0.0, 0.0, 0.0, 180, 0.0, 0.0, "temp_offset"]
y_1 = p[0.0, 120.0, 0.0,  180, 0.0, 0.0, "temp_offset"]
x_1 = p[60.0, 0.0, 0.0,  180, 0.0, 0.0, "temp_offset"]
origin_neg = p[0.0, 0.0, -50.0, 180, 0.0, 0.0, "temp_offset"]
y_1_neg = p[0.0, 120.0, -50.0,  180, 0.0, 0.0, "temp_offset"]
x_1_neg = p[60.0, 0.0, -50.0,  180, 0.0, 0.0, "temp_offset"]
from robot_command.calibration import *

with open(filename, 'w') as csvfile:
    csvwriter = csv.writer(csvfile)
    # write column names
    csvwriter.writerow(['x', 'y', 'z', 'a', 'b', 'c'])

def main():
    change_work_offset("temp_offset")
    notify("Check probe input before continuing...", warning=True)
    movej(origin)
    origin_point = probel(origin_neg, a=.5, v=.01)
    movel(x_1)
    x_point = probel(x_1_neg, a=.5, v=.01)
    movel(y_1)
    y_point = probel(y_1_neg, a=.5, v=.01)
    new_offset = calculate_work_offset_3(origin_point, x_point, y_point)
    # notify("new offset is " + str(new_offset), True)
    with open(filename, 'a+') as csvfile:
        # open in append mode ('a+') so that new data can be added
        csvwriter = csv.writer(csvfile)
        # write probed points
        csvwriter.writerow(origin_point.to_list())
        csvwriter.writerow(x_point.to_list())
        csvwriter.writerow(y_point.to_list())
        # write offset values
        csvwriter.writerow(new_offset.to_list())