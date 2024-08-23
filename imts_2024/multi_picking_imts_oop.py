from robot_command.rpl import *
from geometry_utils import set_and_change_toolframe_from_mat, rot_z
from lidar_tools import fetch_workpiece_from_stack
from pneumatic_tools import open_main_gripper
from object_placer import ObjectPlacer
from tending_settings import load_tending_settings, set_max_layers
import numpy as np
import os
from math import pi
from counter_settings import increment_bin_objects, get_bin_objects, reset_bin_objects, set_bin_objects

# from json_utils import set_layer_1, set_layer_2

set_units("mm", "rad", "s")

def put_object_to_bin(placer: ObjectPlacer):
    part_id = get_bin_objects()
    change_tool_frame("gripper_frame")
    sleep(0.1)
    x, y, z = placer.get_dropoff_position(part_id)
    z_above = 0.250
    movej(p[x, y, z_above, pi, 0, -pi/2])
    movel(p[x, y, z, pi, 0, -pi/2], velocity=0.1)
    open_main_gripper()
    movel(p[x, y, z_above, pi, 0, -pi/2])


biased_flange_frame = np.eye(4) @ rot_z((pi/180) *3)
lidar_sensor_frame = np.array([
    [0, -1, 0, -0.071],
    [1, 0, 0, 0.011],
    [0, 0, 1, 0.122],
    [0, 0, 0, 1]
])
gripper_frame = np.array([
    [0, -1, 0, -0.016],
    [1, 0, 0, -0.009],
    [0, 0, 1, 0.165],
    [0, 0, 0, 1]
])
gripper_frame = biased_flange_frame @ gripper_frame

set_units("m", "rad", "s")
set_and_change_toolframe_from_mat("gripper_frame", gripper_frame)
set_and_change_toolframe_from_mat("lidar_frame", lidar_sensor_frame)
change_user_frame("my_world_frame")

TENDING_SETTINGS_FILENAME = "tending_settings.json"

def main():
    settings = load_tending_settings(TENDING_SETTINGS_FILENAME)
    num_objects_in_bin = get_bin_objects()
    
    user_input = input(f"The system indicates {num_objects_in_bin} finished parts in the bin. Press [enter] to proceed or type a new number of objects in the bin.", default=str(num_objects_in_bin))
    n = int(user_input)
    set_bin_objects(n)

    sleep(0.5)

    current_max_layers = settings.stack_settings.max_layers
    new_max_layers_str = input(f"The system will start scanning layer {current_max_layers}. Press [enter] to proceed or type a new top layer value [1 to 3].", default=str(current_max_layers))
    new_max_layers = int(new_max_layers_str)
    if new_max_layers != current_max_layers:
        set_max_layers(TENDING_SETTINGS_FILENAME, new_max_layers)
        settings = load_tending_settings(TENDING_SETTINGS_FILENAME)

    # settings.stack_settings.max_layers = int(begin_layer)

    placer = ObjectPlacer(settings.placement_settings, settings.ready_part)

    set_units("m", "rad", "s")

    file_path='radial_swipe_poses.csv'

    if os.path.isfile(file_path):
        os.remove(file_path)
        print(f"'{file_path}' has been removed.")

    picking_part_id = 0

    while True:
        if fetch_workpiece_from_stack(part_id=picking_part_id,
                                      settings=settings):
            put_object_to_bin(placer)
            increment_bin_objects()
            picking_part_id += 1
        else:
            break

    exit()
