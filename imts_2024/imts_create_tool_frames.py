import numpy as np
from geometry_utils import set_and_change_toolframe_from_mat
from geometry_utils import set_and_change_toolframe_from_mat, rot_z

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