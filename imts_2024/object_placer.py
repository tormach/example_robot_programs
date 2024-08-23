import math
from tending_settings import load_tending_settings, PlacementSettings, Object

class Object:
    def __init__(self, x: float, y: float, z: float, x_clearance: float, y_clearance: float):
        self.x = x
        self.y = y
        self.z = z
        self.x_clearance = x_clearance
        self.y_clearance = y_clearance

class ObjectPlacer:
    def __init__(self, placement_settings: PlacementSettings, object: Object):
        self.roi_min = placement_settings.roi_min
        self.roi_max = placement_settings.roi_max
        self.layer_translation = placement_settings.layer_translation
        self.object = object
        self.capacity = self.calculate_capacity()

    def calculate_capacity(self):
        roi_size = [
            abs(self.roi_max[0] - self.roi_min[0]),
            abs(self.roi_max[1] - self.roi_min[1]),
            abs(self.roi_max[2] - self.roi_min[2])
        ]
        # Calculate number of objects in each dimension
        objects_x = math.floor((roi_size[0] + self.object.x_clearance) / (self.object.x + self.object.x_clearance))
        objects_y = math.floor((roi_size[1] + self.object.y_clearance) / (self.object.y + self.object.y_clearance))
        objects_z = math.floor(roi_size[2] / self.object.z)
        total_objects = objects_x * objects_y * objects_z
        print(f"Rows (Y): {objects_y}")
        print(f"Columns (X): {objects_x}")
        print(f"Layers (Z): {objects_z}")
        print(f"Total capacity: {total_objects} objects")
        self.objects_per_layer = objects_x * objects_y
        self.objects_x = objects_x
        self.objects_y = objects_y
        self.objects_z = objects_z
        return total_objects

    def get_dropoff_position(self, object_id):
        if object_id >= self.capacity:
            raise ValueError("Object ID exceeds capacity")
        layer = object_id // self.objects_per_layer
        position_in_layer = object_id % self.objects_per_layer
        row = position_in_layer // self.objects_x
        col = position_in_layer % self.objects_x

        # Calculate center positions of the bottom face
        x = self.roi_min[0] + col * (self.object.x + self.object.x_clearance) + self.object.x/2
        y = self.roi_min[1] + row * (self.object.y + self.object.y_clearance) + self.object.y/2
        z = self.roi_min[2] + layer * self.object.z

        # Apply layer translation
        x += layer * self.layer_translation[0]
        y += layer * self.layer_translation[1]

        return (x, y, z)

if __name__ == "__main__":
    # Example usage
    # object = Object(x=0.106, y=0.0508, z=0.0254, x_clearance=0.020, y_clearance=0.035)
    # roi_min = (-0.750, 0.170, 0.085)
    # roi_max = (-0.400, 0.530, 0.210)
    # layer_translation = (0.01, 0.01)  # 1cm translation in both x and y directions for each layer
    # settings = PlacementSettings(roi_min, roi_max, layer_translation)
    settings = load_tending_settings("tending_settings.json")
    placer = ObjectPlacer(settings.placement_settings, settings.ready_part)

    # Get dropoff positions for all objects
    for i in range(placer.capacity):
        print(f"Object {i} bottom-center position: {placer.get_dropoff_position(i)}")