set_units("mm", "deg")

import moveit_commander
scene = moveit_commander.PlanningSceneInterface()


def main():
    scene.remove_attached_object('tool0')
    scene.remove_world_object('saw')
    sleep(1.0)
    exit()