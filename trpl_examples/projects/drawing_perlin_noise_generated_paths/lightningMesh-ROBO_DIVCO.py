# import json
import random
import math
from robot_command.rpl import * 

try:
    from perlin_noise import PerlinNoise
except:
    notify("Installing package")
    # In case the library doesn't exist we install it.
    # (NOTE: Make sure the computer has access to the internet)
    import sh
    # Install packages that are not available
    with sh.sudo:
        sh.pip3.install("perlin_noise")
    from perlin_noise import PerlinNoise

W_WIDTH = 600
W_HEIGHT = 600
MAP_OFFSET_X = 30
MAP_OFFSET_Y = 30

SAFE_HEIGHT = 50
CELL_SCALE = 1  # bigger numbers = less resolution

NUM_PATHS = 600
MAX_PATH_LENGTH = 100   # Maximum points in a path (Too many points might crush the system)
MIN_PATH_LENGTH = 20    # Maximum number of paths in each batch

NUM_BATCH_LENGTH = 3

def LMesh(width=200, height=200):
    notify("Creating Mesh")
    paths = []
    noise = PerlinNoise(octaves=3, seed=777)

    x_offs = MAP_OFFSET_X
    y_offs = MAP_OFFSET_Y

    grid = []  # Vector field
    cell_scl = CELL_SCALE
    cols = int(width / cell_scl)
    rows = int(height / cell_scl)

    # Add grid frame
    paths.append([
                (x_offs, y_offs),
                (width+x_offs, y_offs),
                (width+x_offs, height+y_offs),
                (x_offs, height+y_offs),
                (x_offs, y_offs)
                ])

    #  Generating vector field
    # ------------------------

    noise_z_frame = random.uniform(1, cols*rows)
    xy_noise_scale = random.randint(2, 20)
    # xy_noise_scale = 3

    for r in range(0, rows):
        for c in range(0, cols):
            x = c * cell_scl + x_offs
            y = r * cell_scl + y_offs

            # Get random number form perlin noise, this will be used to rotate vectors
            rnd = noise([(c/width)*xy_noise_scale, (r/height)*xy_noise_scale, noise_z_frame]) * math.pi * 2 * 4
            
            # ---------EXPE---------------------------------------------------
            # rnd = random.uniform(0, math.radians(360))
            # rnd = random.uniform(0, math.radians(360)) % 30 # better with higher cell_scl
            # rnd = random.uniform(0, math.radians(360)) % 2
            # rnd = (noise([(c/width)*xy_noise_scale, (r/height)*xy_noise_scale, noise_z_frame]) * math.pi * 2 * 4) % 2
            # ------------------------------------------------------------
            
            # Rotated x and y
            rX = math.cos(rnd)
            rY = math.sin(rnd)
            x2 = x+cell_scl * rX
            y2 = y+cell_scl * rY

            temp_point = {
                "x": x,
                "y": y,
                "x2": x2,
                "y2": y2,
                "rV": (rX, rY),
                "cx": x + cell_scl/2,
                "cy": y + cell_scl/2,
                "vPath": [(x, y), (x2, y2)],
                "renderCount": 0,
            }

            # paths.append(temp_point["vPath"]) # Add it to paths so it can get graphed
            grid.append(temp_point)

    def draw_rect(_x, _y, _width, _height):
        temp_path = []
        temp_path.append((_x, _y))
        temp_path.append((_x + _width, _y))
        temp_path.append((_x + _width, _y + _height))
        temp_path.append((_x, _y + _height))
        temp_path.append((_x, _y))
        paths.append(temp_path)

    # Trace paths in the vector field
    def trace(_px, _py, index, center=""):  # "center" is an experiment at the moment
        path = []
        px = _px
        py = _py

        margin = 1
        if center == "c":
            margin = cell_scl/2

        while True:
            _x = int(px/cell_scl)
            _y = int(py/cell_scl)
            g_index = _x + _y * cols

            # When current grid index is out of bounds return current path
            if g_index > (rows*cols-1):
                return path
            elif g_index < 0:
                return path
            g_cell = grid[g_index]

            # -----------------
            # When current cell is out of bounds return current path
            #  And if path gets too long or at the edge, end it
            if len(path) > MAX_PATH_LENGTH:
                return path
            if g_cell[center + "x"] < (x_offs + margin):
                return path
            elif g_cell[center + "x"] > (width + x_offs - margin):
                return path
            elif g_cell[center + "y"] < (y_offs + margin):
                return path
            elif g_cell[center + "y"] > (height + x_offs - margin):
                return path
            else:
                path.append((g_cell[center+"x"], g_cell[center+"y"]))  # Add point to current path
                rVec = g_cell["rV"]  # Get rotation vector at current point in grid (vector field)
                # Using rotation vector, calculate next point
                px += rVec[0]
                py += rVec[1]

    # Creating flow field paths
    # --------------------------
    index = 0
    for _ in range(NUM_PATHS):
        # Start from random position
        pos_x = random.uniform(0, cols) * cell_scl
        pos_y = random.uniform(0, rows) * cell_scl

 # From given position, trace a path
        path = trace(pos_x, pos_y, index)
        if len(path) > MIN_PATH_LENGTH:
            paths.append(path)  # Add created path to paths list to be drawn
        index += 1
    print("DONE!!")
    return paths
# END of LMesh()
# --------------------------------------------------------------------------------------


# divco() will divide a list into batches (divide and conquer)
def divco(list):
    if len(list) <= NUM_BATCH_LENGTH:
        return [list]
    else:
        mid = len(list) // 2
        left = list[:mid]
        right = list[mid:]
        return divco(left) + divco(right)


all_paths = LMesh()


# Setting up user frames and moving to home
def init_setup():
    notify("Setting frames")
    set_tool_frame("art_tool_frame", orientation=p[0, 0, 0, 180, 0, 0])
    change_tool_frame("art_tool_frame")

    # Below we create a test user frame that will be above a table,
    # if you want the script to draw on paper or any other medium you
    # will need to set up a user frame for that.
    set_user_frame("art_user_frame_test", position=p[400, -200, 550, 0, 0, 0])
    change_user_frame("art_user_frame_test")
    movej(p[0,0,SAFE_HEIGHT,0,0,0])

# Will run a batch of paths
def start(_paths):
    if not _paths:
        notify("No paths")
        exit()
    x = 0
    y = 0

    # Enable path blending with 0 blend radius
    set_path_blending(True, 0)
    for _path in _paths:  # for every path  in c_paths[] list
        if not _path:
            notify("No path")
            exit()
        for point in _path:
            x = point[0]
            y = point[1]
            movel(p[x, y, 0, 0, 0, 0])
        movel(p[x, y, SAFE_HEIGHT, 0, 0, 0])
    sync()
    set_path_blending(False)

def blend_n_run(batches):
    for batch in batches:
        start(batch)
    # When done, move to home
    movel(p[0, 0, SAFE_HEIGHT, 0, 0, 0])

def main():
    init_setup()
    blend_n_run(divco(all_paths))
    exit()
