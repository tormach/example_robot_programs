import json
from typing import Tuple, NamedTuple

class Object(NamedTuple):
    x: float
    y: float
    z: float
    x_clearance: float
    y_clearance: float

class LidarSettings(NamedTuple):
    z_global_clearance: float
    z_refined_clearance: float
    z_min: float
    z_max: float

class ICPSettings(NamedTuple):
    num_vertices_in_group: float
    cluster_eps: float
    z: float

class StackSettings(NamedTuple):
    max_layers: int
    safe_z_lift: float
    z_right_above: float
    z_torque_check: float
    z_grasp: float

class PlacementSettings(NamedTuple):
    roi_min: Tuple[float, float, float]
    roi_max: Tuple[float, float, float]
    layer_translation: Tuple[float, float]

class TendingSettings:
    def __init__(self, data: dict):
        self.workpiece = Object(**data['workpiece'])
        self.ready_part = Object(**data['ready_part'])
        self.lidar_settings = LidarSettings(**data['lidar_settings'])
        self.icp_settings = ICPSettings(**data['icp_settings'])
        self.stack_settings = StackSettings(**data['stack_settings'])
        self.placement_settings = PlacementSettings(**data['placement_settings'])
        self.global_scan_after_layer = data['global_scan_after_layer']

def load_tending_settings(file_path: str) -> TendingSettings:
    with open(file_path, 'r') as file:
        data = json.load(file)
    return TendingSettings(data)

def print_tending_settings(settings: TendingSettings, indent: str = ""):
    def print_object(obj, name, indent):
        print(f"{indent}{name}:")
        for field in obj._fields:
            value = getattr(obj, field)
            print(f"{indent}  {field}: {value}")

    print(f"{indent}TendingSettings:")
    print(f"{indent}  Workpiece:")
    print_object(settings.workpiece, "Workpiece", indent + "    ")
    print(f"{indent}  Ready Part:")
    print_object(settings.ready_part, "Ready Part", indent + "    ")
    print(f"{indent}  Lidar Settings:")
    print_object(settings.lidar_settings, "Lidar Settings", indent + "    ")
    print(f"{indent}  ICP Settings:")
    print_object(settings.icp_settings, "ICP Settings", indent + "    ")
    print(f"{indent}  Stack Settings:")
    print_object(settings.stack_settings, "Stack Settings", indent + "    ")
    print(f"{indent}  Placement Settings:")
    print_object(settings.placement_settings, "Placement Settings", indent + "    ")
    print(f"{indent}  Global Scan After Layer: {settings.global_scan_after_layer}")

def set_max_layers(filename: str, number: int):
    with open(filename, 'r+') as file:
        data = json.load(file)
        data['stack_settings']['max_layers'] = number
        file.seek(0)
        json.dump(data, file, indent=2)
        file.truncate()


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) != 2:
        print("Usage: python script.py <path_to_json_file>")
        sys.exit(1)
    
    json_file_path = sys.argv[1]
    try:
        tending_settings = load_tending_settings(json_file_path)
        print("TendingSettings object initialized successfully.")
        print("\nPrinting TendingSettings:")
        print_tending_settings(tending_settings)
    except Exception as e:
        print(f"Error: Failed to load TendingSettings. {str(e)}")
        sys.exit(1)
