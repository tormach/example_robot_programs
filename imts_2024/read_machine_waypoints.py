import yaml
from collections import namedtuple
from typing import Dict

# Define the namedtuples
Position = namedtuple('Position', ['x', 'y'])
Waypoint = namedtuple('Waypoint', ['name', 'position', 'feedrate', 'description'])

def read_machine_waypoints(file_path: str) -> Dict[str, Waypoint]:
    """
    Read waypoints from a YAML file and return a dictionary of Waypoint namedtuples.

    :param file_path: Path to the YAML file
    :return: Dictionary of Waypoint namedtuples with names as keys
    """
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)

    waypoints = {}
    for wp in data['waypoints']:
        position = Position(**wp['position'])
        waypoint = Waypoint(
            name=wp['name'],
            position=position,
            feedrate=wp['feedrate'],
            description=wp['description']
        )
        waypoints[wp['name']] = waypoint

    return waypoints

if __name__ == "__main__":
    waypoints = read_machine_waypoints("machine_waypoints.yaml")

    # Print all waypoints
    # for name, wp in waypoints.items():
        # print(f"Name: {name}")
        # print(f"Position: x={wp.position.x}, y={wp.position.y}, z={wp.position.z}")
        # print(f"Feedrate: {wp.feedrate}")
        # print(f"Description: {wp.description}")
        # print("---")

    # Example: Accessing a specific waypoint by name
    start_point = waypoints["start_point"]
    print("Accessing 'Start Point':")
    print(f"Name: {start_point.name}")
    print(f"Position: x={start_point.position.x}, y={start_point.position.y}")
    print(f"Feedrate: {start_point.feedrate}")
    print(f"Description: {start_point.description}")
    print("---")
