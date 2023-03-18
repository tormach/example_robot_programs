import os
import pygame
import json
import random
import math
from perlin_noise import PerlinNoise


W_WIDTH = 600 # World/Canvas width
W_HEIGHT = 600 # World/Canvas height

MAP_OFFSET_X = 30
MAP_OFFSET_Y = 30
NUM_PATHS = 1000
CELL_SCALE = 1
MAX_PATH_LENGTH = 100   # Maximum number of points in each path
MIN_PATH_LENGTH = 20    # Maximum number of paths in each batch

PREVIEW_EXISTING_JSON = False
CREATE_JSON_FILE = True

EXISTING_JSON_FILE = "paths/pathsMap_extract.json"
SAVE_JSON_FILE_PATH = "paths/pathsMap_extract.json"


'''
LMesh(): Handles mesh generation (Lighting mesh generator) and
returns path list
NOTE: Keep width and height below 500, above that the robot might not reach to draw paths
'''
def LMesh(width=200, height=200):
    paths = []
    noise = PerlinNoise(octaves=3, seed=777)

    x_offs = MAP_OFFSET_X
    y_offs = MAP_OFFSET_Y

    grid = []  # Vector field
    cell_scl = CELL_SCALE
    cols = int(width / cell_scl)
    rows = int(height / cell_scl)

    # Add grid frame
    paths.append([(x_offs, y_offs),
                  (width+x_offs, y_offs),
                  (width+x_offs, height+y_offs),
                  (x_offs, height+y_offs),
                  (x_offs, y_offs)
                  ])

    #  Generating vector field
    # ------------------------

    noise_z_frame = random.uniform(1, cols*rows)
    xy_noise_scale = random.randint(2, 20)
    # xy_noise_scale = 2
    # print(noise_z_frame)
    for r in range(0, rows):
        for c in range(0, cols):
            # Get xy coordinate of cell (top left corner)
            x = c * cell_scl + x_offs
            y = r * cell_scl + y_offs

            # Get random number form perlin noise, this will be used to rotate vectors
            rnd = noise([(c/width)*xy_noise_scale, (r/height)*xy_noise_scale, noise_z_frame]) * math.pi * 2 * 4
            # ---------------Experimental---------------------------------------------
            # rnd = random.uniform(0, math.radians(360))
            # rnd = random.uniform(0, math.radians(360)) % 30 # better with higher cell scale
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

            # paths.append(temp_point["vPath"])  # Add to paths to be drawn
            grid.append(temp_point)

    def draw_rect(_x, _y, _width, _height):
        temp_path = []
        temp_path.append((_x, _y))
        temp_path.append((_x + _width, _y))
        temp_path.append((_x + _width, _y + _height))
        temp_path.append((_x, _y + _height))
        temp_path.append((_x, _y))
        paths.append(temp_path)  # Add to paths to be drawn

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
            if g_index > (rows*cols - 1):
                return path
            elif g_index < 0:
                return path

            g_cell = grid[g_index]

            # -----------------
            # When current cell is out of bounds return current path
            #  And if path gets too long or at the edge, end it
            if len(path) > MAX_PATH_LENGTH:
                return path
            if g_cell[center+"x"] < (x_offs + margin):
                return path
            elif g_cell[center+"x"] > (width + x_offs - margin):
                return path
            elif g_cell[center+"y"] < (y_offs + margin):
                return path
            elif g_cell[center+"y"] > (height + y_offs - margin):
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
            print("Loading:", index+1, "/", NUM_PATHS)
        index += 1
        # print(path)
    print("DONE!!")
    return paths
# END of LMesh()
# --------------------------------------------------------------------------------------

def get_existing_JSON(file_path=EXISTING_JSON_FILE):
    with open(file_path, "r") as openFile:
        json_data = json.load(openFile)
    return json_data["paths"]


all_paths = []
if (not PREVIEW_EXISTING_JSON):
    all_paths = LMesh()  # Generate paths
else:
    all_paths = get_existing_JSON()


if (CREATE_JSON_FILE and not PREVIEW_EXISTING_JSON):
    json_data = json.dumps({
        "paths": all_paths
    })
    # Save data to JSON file
    with open(SAVE_JSON_FILE_PATH, "w+") as outfile:
        outfile.write(json_data)


pygame.init()  # init pygame
surface = pygame.display.set_mode(
    (W_WIDTH, W_HEIGHT)
)  # get surface to draw on (width = 600, height = 600)
surface.fill(pygame.Color("white"))  # set background to white

for _path in all_paths:  # for every path  in c_paths[] list
    pygame.draw.aalines(
        surface, pygame.Color("blue"), False, _path
    )  # False is no closing (Meaning a closing line wont be drawn from the last point to the first)
    pygame.display.update()  # copy surface to display


while True:  # loop to wait till window close
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            exit()
