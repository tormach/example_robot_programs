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

def load_and_check_tending_settings():
    set_units("m", "rad", "s")
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

    return settings


class Globals():
    def __init__(self):
        self.first_run = True
        self.part_count = 0

g = Globals()

POSES_FILE_PATH='radial_swipe_poses.csv'

def main():
    if g.first_run == True:
        sanity_check()

        if os.path.isfile(POSES_FILE_PATH):
            os.remove(POSES_FILE_PATH)
            print(f"'{POSES_FILE_PATH}' has been removed.")

            settings = load_and_check_tending_settings()
    else:
        #TODO: only load settings
        settings = load_tending_settings()

    g.part_count += 1
    nofity(f"part_count: {g.part_count}", warning=True)
    sleep(1)

    # placer = ObjectPlacer(settings.placement_settings, settings.ready_part)
    # # settings.stack_settings.max_layers = int(begin_layer)

    # set_units("m", "rad", "s")

    # if fetch_workpiece_from_stack(part_id=g.part_count, settings=settings):
        # put_object_to_bin(placer)
        # increment_bin_objects()
        # g.part_count += 1
    # else:
        # break

    # exit()
