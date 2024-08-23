import json

def increment_bin_objects():
    with open('counter_settings.json', 'r+') as file:
        data = json.load(file)
        data['counter_settings']['processed_objects'] += 1
        data['counter_settings']['total_processed_objects'] += 1
        file.seek(0)
        json.dump(data, file, indent=2)
        file.truncate()

def set_bin_objects(number: int):
    with open('counter_settings.json', 'r+') as file:
        data = json.load(file)
        data['counter_settings']['processed_objects'] = number
        file.seek(0)
        json.dump(data, file, indent=2)
        file.truncate()

def reset_bin_objects():
    with open('counter_settings.json', 'r+') as file:
        data = json.load(file)
        data['counter_settings']['processed_objects'] = 0
        file.seek(0)
        json.dump(data, file, indent=2)
        file.truncate()

def get_bin_objects():
    with open('counter_settings.json', 'r') as file:
        data = json.load(file)
        return data['counter_settings']['processed_objects']
