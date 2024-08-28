def read_int_from_file(filename):
    try:
        with open(filename, 'r') as file:
            content = file.read().strip()
            return int(content)
    except FileNotFoundError:
        print(f"Error: File '{filename}' not found.")
        return None
    except ValueError:
        print(f"Error: Content in '{filename}' is not a valid integer.")
        return None

def write_int_to_file(value, filename):
    try:
        with open(filename, 'w') as file:
            file.write(str(value))
        print(f"Successfully wrote {value} to '{filename}'.")
    except IOError:
        print(f"Error: Unable to write to file '{filename}'.")
