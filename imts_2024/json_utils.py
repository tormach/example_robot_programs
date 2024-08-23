import json

def set_max_layers(filename: str, layers: int):
    """
    Modify the max_layers setting in the JSON configuration file.

    Args:
    filename (str): Path to the JSON configuration file.
    layers (int): Number of layers to set.

    Raises:
    ValueError: If layers is less than 1.
    """
    if layers < 1:
        raise ValueError("Number of layers must be at least 1")

    with open(filename, 'r') as file:
        config = json.load(file)

    config['stack_settings']['max_layers'] = layers

    with open(filename, 'w') as file:
        json.dump(config, file, indent=2)

def set_layer_2(filename: str):
    """Set max_layers to 2 in the specified JSON configuration file."""
    set_max_layers(filename, 2)

def set_layer_1(filename: str):
    """Set max_layers to 1 in the specified JSON configuration file."""
    set_max_layers(filename, 1)
