from robot_command.rpl import *

from pose_validator import validate_pose
from lidar_tools import fetch_workpiece_from_stack
from tending_settings import load_tending_settings, set_max_layers
from counter_settings import increment_bin_objects, get_bin_objects, reset_bin_objects, set_bin_objects
from pneumatic_tools import open_main_gripper
from geometry_utils import set_and_change_toolframe_from_mat, rot_z
from read_machine_waypoints import read_machine_waypoints
from robot_machine_watchdog_handler import RobotMachineWatchdogHandler
from geometry_msgs.msg import Pose as ROSPose
from robot_command.interfaces import MachinetalkInterfaceSingleton
from object_placer import ObjectPlacer
from math import pi
import numpy as np
import os

cnc_handler = RobotMachineWatchdogHandler()

set_units("mm", "deg", "s")

# pose waypoints for grasping
DISTANCE_ABOVE_PART = 50
CLEAR_DISTANCE = 200
TENDING_SETTINGS_FILENAME = "tending_settings.json"
POSES_FILE_PATH='radial_swipe_poses.csv'
MACHINE_WAYPOINTS_FILE = "machine_waypoints.yaml"

cnc_waypoints = read_machine_waypoints(MACHINE_WAYPOINTS_FILE)

home_wpt = cnc_waypoints["home_wpt"]
before_op1_pick_wpt = cnc_waypoints["before_op1_pick_wpt"]
op1_pick_wpt = cnc_waypoints["op1_pick_wpt"]
op2_pick_wpt = cnc_waypoints["op2_pick_wpt"]
op2_before_place_wpt = cnc_waypoints["op2_before_place_wpt"]
op2_place_wpt = cnc_waypoints["op2_place_wpt"]
op2_after_place_wpt = cnc_waypoints["op2_after_place_wpt"]

# joint waypoints to help get in and out of the enclosure
vision_start = j[-0.000, 17.933, 6.173, 0.000, 65.895, -0.000]
post_check = j[15.689, -18.091, 42.153, 0.469, 64.559, 103.559]
pre_window = j[-79.136, -19.011, 42.736, 0.511, 65.069, 7.933]
through_window = j[-72.309, 43.924, 11.742, -75.220, 99.598, 55.739]
post_window = j[-84.366, 14.224, 11.609, 0.863, 62.887, 2.140]
vision_pre_grasp = p[0, 0, DISTANCE_ABOVE_PART, -179.995, 0, 0, NUT, 0]
safe_op1_transition = j[-79.377, 33.555, -4.596, -0.047, 60.432, 9.919]
safe_op2_transition = j[-74.521, 37.964, 24.494, 104.293, 82.363, -65.028]
flipping_grasp = p[0, 0, 0, 0, 0, 0, NUT, 0]
flipping_grasp_vise_2 = p[0, 0, 0, -180, 0, 0, NUT, 0]

biased_flange_frame = np.eye(4) @ rot_z((pi/180) *3)
lidar_sensor_frame = np.array([
    [0, -1, 0, -0.071],
    [1, 0, 0, 0.011],
    [0, 0, 1, 0.122],
    [0, 0, 0, 1]
])
lidar_gripper_frame = np.array([
    [0, -1, 0, -0.016],
    [1, 0, 0, -0.009],
    [0, 0, 1, 0.165],
    [0, 0, 0, 1]
])
lidar_gripper_frame = biased_flange_frame @ lidar_gripper_frame

set_units("m", "rad", "s")
set_and_change_toolframe_from_mat("lidar_gripper_frame", lidar_gripper_frame)
set_and_change_toolframe_from_mat("lidar_frame", lidar_sensor_frame)
change_user_frame("my_world_frame")

class Globals():
    def __init__(self):
        self.first_run = True
        self.part_count = 0

g = Globals()

def main():
    if g.first_run == True:
        g.first_run = False
        if os.path.isfile(POSES_FILE_PATH):
            os.remove(POSES_FILE_PATH)
            print(f"'{POSES_FILE_PATH}' has been removed.")

        settings = load_and_check_tending_settings()


    settings = load_tending_settings(TENDING_SETTINGS_FILENAME)
    placer = ObjectPlacer(settings.placement_settings, settings.ready_part)
        #TODO: only load settings

    set_units("m", "rad", "s")
    is_part_picked = fetch_workpiece_from_stack(part_id=g.part_count, settings=settings)

    if is_part_picked != True:
        notify("Failed to find a part.  Retry?", warning=True)

    while get_pathpilot_state(instance="1500MX") != "ready":
        sync()
        sleep(.5)


    # check probe result
    # aout_60 is general "probe returned error" value.  -1.0 is error, 0.0 is OK
    aout_60, aout_61 = get_workcell_reported_state()
    if aout_60 == -1.0:
        notify("Mill program has aborted because the probe routine shows that the workpiece is out of position.  Fix and click OK to continue or abort.", warning=True)

    set_units("mm", "deg", "s")
    move_into_mill()
    movej(through_window, velocity_scale=1.0)
    pick_from_op1_vise()
    place_in_op1_vise()
    pick_from_op2_vise()
    place_in_op2_vise()
    move_out_of_mill()
    pathpilot_cycle_start(instance="1500MX")
    put_object_to_bin(placer)
    increment_bin_objects()
    g.part_count += 1


def move_into_mill():
    # should be left in the post_check position by the check_part method, but for testing and convenience, let's go there
    change_tool_frame("lidar_gripper_frame")

    limits_low = (300, -50, 170, -170, -50, -120.0)
    limits_high = (800, 550, 900, 170, 50, 50.0)

    if not validate_pose(limits_low, limits_high):
        notify("Robot is not in a valid position to move into mill.  Please fix and restart.", error=True)

    set_units("mm", "deg", "s")
    movej(post_check)
    # start by opening door
    open_door()
    movej(pre_window)
    movej(post_window)
    # movel(through_window)
    pass


def move_out_of_mill():
    # start well above workpiece
    # todo: get Z position in base frame, then move linearly in Z to some clearance plane before anything else

    # active_tool_frame = get_active_tool_frame()
    # if active_tool_frame == "lidar_gripper_frame":
    #     limits_low = (400, -50, 800, -170, -10, -21.0)
    #     limits_high = (500, 50, 900, 170, 10, 21.0)
    # elif active_tool_frame == "flipping":
    #     limits_low = (400, -50, 800, -170, -10, -21.0)
    #     limits_high = (500, 50, 900, 170, 10, 21.0)
    # else:
    #     notify("Unknown tool frame active.  Please change to gripper_frame or flipping and restart.", error=True)

    # if not validate_pose(limits_low, limits_high):
    #     notify("Robot is not in a valid position to move into mill.  Please fix and restart.", error=True)

    set_units("mm", "deg", "s")
    movej(post_window)
    movel(pre_window)
    movej(post_check)
    close_door()
    pass


def pick_from_op1_vise():
    # using flipping gripper
    # flipping gripper moves to 0, then mill does the work
    set_units("mm", "deg", "s")
    change_tool_frame("flipping")
    
    # open gripper in case it's not already
    set_digital_out("flipping_gripper", True)
    # picking from op1 vise
    change_user_frame("vise_1_flipping")
    
    # move mill into position, no need to check torques
    move_mill(wpt=home_wpt.position, feedrate=home_wpt.feedrate)

    # need to have 2.5mm clearance to slide gripper into position
    temp_wp = flipping_grasp.copy()
    temp_wp.z += 2.5
    movej(temp_wp)
    
    # move will relatively quickly to a place where we're about to engage the grippper
    move_mill_safely(wpt=before_op1_pick_wpt.position, feedrate=before_op1_pick_wpt.feedrate)
    
    # now take our time
    move_mill_safely(wpt=op1_pick_wpt.position, feedrate=op1_pick_wpt.feedrate)

    # down a bit to engage top of gripper
    movel(flipping_grasp, velocity=25, accel_scale=0.5)
    
    # close gripper and open vise
    close_gripper()
    open_vise("vise_1")
    
    # move to up and away to clear distance
    movei(z=CLEAR_DISTANCE, vel=50, accel_scale=0.5)
    movej(safe_op1_transition)


def place_in_op1_vise():
    # using lidar_gripper
    set_units("mm", "deg", "s")
    change_tool_frame("lidar_gripper_frame")
    # placing in op1 vise
    change_user_frame("vise_1_vision")

    # these next four instructions are redundant if pick_from_op1_vise is called first
    movej(safe_op1_transition)
    move_mill(wpt=op1_pick_wpt.position, feedrate=op1_pick_wpt.feedrate)
    # make sure vise_2 is closed so vise_1 has room to open and don't sleep on it
    set_digital_out("vise_2", False) 
    set_digital_out("vise_1", True)
    
    movej(vision_pre_grasp, velocity_scale=1.0)
    movel(vision_pre_grasp)
    movei(z=-(DISTANCE_ABOVE_PART+2), vel=40, accel_scale=0.5)
    
    # open gripper and close vise
    open_gripper()
    close_vise("vise_1")

    # linear move to clear position
    movel(vision_pre_grasp, velocity=50, accel_scale=0.5)


def pick_from_op2_vise():
    # using lidar_gripper, so we approach from above
    set_units("mm", "deg", "s")
    change_tool_frame("lidar_gripper_frame")
    # picking from op2 vise, user frames identical save for Z height, may change if Jason levels vise 2
    change_user_frame("vise_2_vision")

    # gripper should already be open, but just in case...
    set_digital_out("lidar_gripper", True)

    move_mill(wpt=op2_pick_wpt.position, feedrate=op2_pick_wpt.feedrate)

    tmp = vision_pre_grasp.copy()
    tmp.z = 0
    movel(tmp, velocity=40, accel_scale=0.5)
    
    # vise one should already be closed, but in case it's not, vise 2 needs room to open so make sure
    set_digital_out("vise_1", False) 
    close_gripper()
    open_vise("vise_2")
    # linear move to clear position
    movel(vision_pre_grasp, velocity=50, accel_scale=0.5)
    movei(z=100)


def place_in_op2_vise():
    set_units("mm", "deg", "s")
    # using flipping gripper
    # flipping gripper moves to clear, then 0, then opens, then a slight clearance move, then mill does the work
    change_tool_frame("flipping")
    change_user_frame("vise_2_flipping")

    # get mill out of the way to give robot more room
    move_mill(wpt=op2_before_place_wpt.position, feedrate=op2_before_place_wpt.feedrate)

    movej(safe_op2_transition)
    move_mill(wpt=op2_place_wpt.position, feedrate=op2_place_wpt.feedrate)

    temp_wp = flipping_grasp_vise_2.copy()
    temp_wp.z += DISTANCE_ABOVE_PART
    movej(temp_wp)
    movel(flipping_grasp_vise_2, velocity=40, accel_scale=0.5)
    open_gripper()
    movei(z=-1.0)
    close_vise("vise_2")
    # move to clear distance
    move_mill_safely(wpt=op2_after_place_wpt.position, feedrate=op2_after_place_wpt.feedrate)


# UTILITIES

def open_gripper():
    gripper_name = get_active_tool_frame()
    if gripper_name == "lidar_gripper_frame":
        set_digital_out("lidar_gripper", True)
    else:
        set_digital_out("flipping_gripper", True)
    sleep(0.5)


def close_gripper():
    gripper_name = get_active_tool_frame()
    # set_digital_out(gripper_name, False)

    gripper_name = get_active_tool_frame()
    if gripper_name == "lidar_gripper_frame":
        set_digital_out("lidar_gripper", False)
    else:
        set_digital_out("flipping_gripper", False)
    sleep(0.5)




def open_vise(vice_name):
    set_digital_out(vice_name, True)
    sleep(1)


def close_vise(vice_name):
    set_digital_out(vice_name, False)
    sleep(1)


def movel2(x=0, y=0, z=0, vel=20):
    # version of movel that keeps abc rotations constant
    current_pose = get_pose()
    movel(p[x, y, z, current_pose.a, current_pose.b, current_pose.c])


def movei(x=0, y=0, z=0, vel=200, accel_scale=1.0):
    # version of movel that keeps abc rotations constant and moves incrementally in any given xyz direction
    current_pose = get_pose()
    x += current_pose.x
    y += current_pose.y
    z += current_pose.z
    movel(p[x, y, z, current_pose.a, current_pose.b, current_pose.c])


def open_door():
    timeout = 0
    set_digital_out("door_close", True)
    sleep(1)
    set_digital_out("door_open", True)
    sleep(1)
    set_digital_out("door_close", False)
    while get_digital_in("door_open_sensor") is False:
        sleep(.25)
        timeout += 0.25
        if timeout > 5:
            notify(
                "Door failed to open fully.  Abort or fix and continue.", warning=True)
    set_digital_out("door_open", False)


def close_door():
    timeout = 0
    set_digital_out("door_open", True)
    sleep(1)
    set_digital_out("door_close", True)
    sleep(1)
    set_digital_out("door_open", False)
    while get_digital_in("door_close_sensor") is False:
        sleep(.25)
        timeout += 0.25
        if timeout > 5:
            notify(
                "Door failed to close fully.  Abort or fix and continue.", warning=True)
    set_digital_out("door_close", False)


def issue_mdi(command):
    pathpilot_mdi(command, instance="1500MX")
    sleep(0.5)
    while get_pathpilot_state(instance="1500MX") != "ready":
        sync()
        sleep(.5)


def get_workcell_reported_state(instance=None):
    status_sub = MachinetalkInterfaceSingleton().get_status(instance)
    if not status_sub.synced:
        return (0, 0)
    # now check the status
    aout_60 = status_sub.motion.aout[60]
    aout_61 = status_sub.motion.aout[61]
    return (aout_60, aout_61)


def put_object_to_bin(placer: ObjectPlacer):
    set_units("m", "rad", "s")
    part_id = get_bin_objects()
    change_user_frame("my_world_frame")
    change_tool_frame("lidar_gripper_frame")
    sleep(0.1)
    x, y, z = placer.get_dropoff_position(part_id)
    z_above = 0.250
    movej(p[x, y, z_above, pi, 0, -pi/2])
    movel(p[x, y, z, pi, 0, -pi/2], velocity=0.1)
    open_main_gripper()
    movel(p[x, y, z_above, pi, 0, -pi/2])


def move_mill_safely(wpt, feedrate, move_type="G59 G1"):
    """
    Wrapper function for issue_mdi to move the machine to a specified waypoint.

    Args:
    wpt (str): The waypoint coordinates (e.g., "X0 Y0")
    move (str, optional): The movement command. Defaults to "G59 G1".

    Returns:
    None
    """

    MACHINE_ROBOT_TORQUE_THRESHOLD = 30

    sleep(1.5)
    if not cnc_handler.start_watchdog(MACHINE_ROBOT_TORQUE_THRESHOLD):
        notify("Failed to start watchdog. Please fix and restart.", error=True)
        exit()

    command = f"{move_type} X{wpt.x} Y{wpt.y} F{feedrate}"
    issue_mdi(command)

    if not cnc_handler.check_safety_status():
        notify("Collision with machine move detected. Please fix and restart.", error=True)

    if not cnc_handler.stop_watchdog():
        notify("Failed to stop watchdog. Please fix and restart.", error=True)

def move_mill(wpt, feedrate, move_type="G59 G1"):
    command = f"{move_type} X{wpt.x} Y{wpt.y} F{feedrate}"
    issue_mdi(command)


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
