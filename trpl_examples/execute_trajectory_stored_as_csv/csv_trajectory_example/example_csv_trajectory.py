set_units("mm", "deg")

def main():
    trajectory = load_trajectory('test.csv')
    execute_trajectory(trajectory)
    sleep(0.5)