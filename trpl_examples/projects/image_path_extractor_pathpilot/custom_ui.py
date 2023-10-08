from robot_command.rpl import *
set_units("mm", "deg", "second")
import rospy
import base64
import json

import os
import time


try:
    import cv2
    import numpy as np
    from svgpathtools import (
        svg2paths,
    )  # , Path, Line, QuadraticBezier, CubicBezier, Arc,

    import scipy.interpolate as interpolate
except:
    set_param("installing_packages", True)
    notify("Installing packages!")
    rospy.logwarn("INST-start:: " + str(time.strftime("%H:%M:%S", time.localtime())))
    # Install packages that are not available
    import sh

    with sh.sudo:
        sh.pip3.install("opencv-python")
        # sh.pip3.install("cv2") # Not working ERROR: Could not find a version that satisfies the requirement cv2 (from versions: none) ERROR: No matching distribution found for cv2
        # sh.pip3.install("numpy")
        sh.pip3.install("svgpathtools")
        sh.pip3.install("scipy")
    import cv2
    from svgpathtools import (
        svg2paths,
    )  # , Path, Line, QuadraticBezier, CubicBezier, Arc,
    import scipy.interpolate as interpolate

    rospy.logwarn("INST-end:: " + str(time.strftime("%H:%M:%S", time.localtime())))
    set_param("installing_packages", False)

USER_FRAME = "drawing_user_frame"

# SUPPORTED_VIEW_FILE_EXT = [".json", ".png", ".jpeg", ".jpg", ".svg"]
SUPPORTED_VIEW_FILE_EXT = [".json", ".png", ".jpeg", ".jpg",]  # ".svg" is removed for now till loading issues are fixed




# MAX_WIDTH = 500
MAX_WIDTH = 250
MIN_WIDTH = 150

INIT_THRESHOLD_1 = 50
INIT_THRESHOLD_2 = 50
MAX_THRESHOLD = 500
MIN_THRESHOLD = 1


NUM_BATCH_LENGTH = 3
MAX_PATH_LENGTH = 100  # points in a path (Too many points might crush the program)
# MIN_PATH_LENGTH = 20

SAFE_HEIGHT = 50
# DEFAULT_IMAGE = "sect.png"
DEFAULT_IMAGE = "./assets/tormach.png"

curr_width = 0
curr_height = 0

image_view = None # For Zoom
temp_image = None # Rendered on preview
image = None

zoom_scale = MIN_WIDTH

paths_map = []

SMOOTH = True

view_files = {"files": []}

test_json = {
    "files": [
        {"name": "test1", "path": "/test", "isFolder": "true", "ext": "none"},
        {"name": "test2", "path": "/test/b.png", "isFolder": "false", "ext": ".png"},
        {"name": "test2", "path": "/test/c.png", "isFolder": "false", "ext": ".png"},
    ]
}


set_param("test_file_list", test_json)
set_param("view_files", view_files)


def log(msg, type="warning"):
    if type == "warning":
        rospy.logwarn(msg)
    if type == "error":
        rospy.logerr(msg)


log("****REMINDER*******\DON'T FORGET TO SETUP A USER FRAME")
# --------------------------------------------------------------------------------------
"""
*************Extractor************
extract(): 
    *image
    *canny_threshold1
    *canny_threshold2
"""


def extract(
    _image, canny_threshold1=INIT_THRESHOLD_1, canny_threshold2=INIT_THRESHOLD_2, add_smoothing=False
):
    log("Getting ready to extract...")

    if len(_image) <= 0:
        log("extract(): Image issues")
        return

    def cv_contour_list_to_pathsList(contourList):
        newList = []
        list = contourList
        for item in list:
            tempList = []
            for point in item:
                newPoint = (float(point[0][0]), float(point[0][1]))
                # print(newPoint)
                tempList.append(newPoint)
            if len(tempList) > 3:
                newList.append(tempList)
        return newList

    # Mirror the image horizontally
    _image = cv2.flip(_image, 1)

    # Canny Edge Detection
    img_blur = _image
    img_blur = cv2.GaussianBlur(_image, (5, 5), 0)

    img_gray = cv2.Canny(
        image=img_blur, threshold1=canny_threshold1, threshold2=canny_threshold2
    )

    # apply binary thresholding
    ret, thresh = cv2.threshold(img_gray, 150, 255, cv2.THRESH_BINARY)

    # visualize the binary image
    # cv2.imshow('Binary image', thresh)
    # cv2.waitKey(0)
    # cv2.imwrite('image_thres1.jpg', thresh)
    # cv2.destroyAllWindows()

    # detect the contours on the binary image using cv2.CHAIN_APPROX_NONE
    contours, _ = cv2.findContours(
        image=thresh, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE
    )

    # ----------
    non_mirror_img = cv2.flip(thresh, 1)
    non_mirror_contours, _ = cv2.findContours(
        image=non_mirror_img, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE
    )
  
    final_paths = cv_contour_list_to_pathsList(contours)
    non_mirror_final_paths = cv_contour_list_to_pathsList(non_mirror_contours)

    log("Extract Done!!")

    image_copy = _image.copy()

    if add_smoothing:  # Apply b-spline path smoother
        final_paths = smooth_all_paths(final_paths)
        # Draw smooth paths or contours on the copy image
        for _path in final_paths:
            pts = np.array(_path, np.int32)
            pts = pts.reshape((-1, 1, 2))
            cv2.polylines(image_copy, [pts], False, (0, 255, 0), thickness=1, lineType=cv2.LINE_AA)
    else:
        # draw contours on the copy image
        image_copy = cv2.drawContours(
            image=image_copy,
            contours=contours,
            contourIdx=-1,
            color=(0, 255, 0),
            thickness=1,
            lineType=cv2.LINE_AA,
        )

    # Mirror the image horizontally
    image_copy = cv2.flip(image_copy, 1)

    return image_copy, final_paths, non_mirror_final_paths


# --------End of Extractor--------------------------------------------------------------------


def extract_svg_paths(file_path, path_steps=1000, scale=1):
    paths, attributes = svg2paths(file_path)  # Getting paths and attributes
    _paths = []  # final paths
    path_count = 0
    for path in paths:
        path_count += 1
        set_param("svg_reader_status", str(path_count) + "/" + str(len(paths)))
        path_points = []
        for i in range(0, path_steps + 1):  # loop frome 0 to path_steps + 1
            p = i / path_steps  # Remapping 0 - path_steps(100) to  0.0 to 1.0
            complex_point = path.point(
                p
            )  # path.point(p) returns point at 0.0 <= p <= 1.0
            path_points.append(
                (complex_point.real * scale, complex_point.imag * scale)
            )  # Adding extracted points to path_point[] list
        _paths.append(path_points)  # Adding collected path points to c_paths[] list
    return _paths


# *******************************
#   Support/Common functions
# *****************************

# log(get_param("extructReq"))


def split_file_path(file_path="/"):
    final_list = ["/"]
    split_path = list(filter(None, file_path.split("/")))
    if len(split_path):
        final_list = final_list + split_path

    return final_list


def join_file_path_from_list(list):
    joined_path = ""
    for i in range(len(list)):
        if i > 0 and i < len(list) - 1:
            joined_path += list[i] + "/"
        else:
            joined_path += list[i]
    return joined_path


def back_trace_file_path(file_path):
    split_path = split_file_path(file_path)
    length = len(split_path)
    final_file_path = ""
    if len(split_path) > 1:
        if split_path[length - 1] == "/":
            split_path.pop(length - 2)
            final_file_path = join_file_path_from_list(split_path)
        else:
            split_path.pop()
            final_file_path = join_file_path_from_list(split_path)
    else:
        final_file_path = split_path[0]
    return final_file_path


def pathsList_to_cv_image(paths):
    img = np.zeros((1000, 1000, 3), np.uint8)
    for path in paths:
        # convert the path to a NumPy array of integers
        path = np.array(path, np.int32)
        # draw the path on the image
        cv2.polylines(img, [path], True, (0, 255, 0), thickness=2)
    # cv2.imshow('image', img)
    return img


def get_existing_pathsList_JSON(file_path="pathsMap_extract.json"):
    with open(file_path, "r") as openFile:
        json_data = json.load(openFile)
    return json_data["paths"]


def cv_image_to_base64(image):
    # Convert the image to JPEG format
    _, buffer = cv2.imencode(".jpg", image)
    # Encode the JPEG buffer as base64
    base64_image = base64.b64encode(buffer).decode("utf-8")
    return base64_image


def scale_cv_image_WH(_image, scale_w=1):
    if _image is None:
        log("[scale_cv_image()] Image is None ")
        return
    height, width, _ = _image.shape
    _ratio = width / height
    if width > height:
        _ratio = height / width

    if width > height:
        new_w = scale_w
        new_h = int(new_w * _ratio)
    else:
        new_h = scale_w
        new_w = int(new_h * _ratio)

    # log("W:H - "+str(width)+", "+str(height)+" RS: "+str(new_w)+", "+str(new_h)+" ratio:"+ str(_ratio))
    # self.set_image_width_height_labels(new_w, new_h)

    _image = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_AREA)
    return _image

def zoom_image(_image, scale_w=1):
    if _image is None:
        log("[scale_cv_image()] Image is None ")
        return
    height, width, _ = _image.shape
    _ratio = width / height
    if width > height:
        _ratio = height / width

    if width > height:
        new_w = scale_w
        new_h = int(new_w * _ratio)
    else:
        new_h = scale_w
        new_w = int(new_h * _ratio)

    # log("W:H - "+str(width)+", "+str(height)+" RS: "+str(new_w)+", "+str(new_h)+" ratio:"+ str(_ratio))
    # self.set_image_width_height_labels(new_w, new_h)

    _image = cv2.resize(_image, (new_w, new_h), interpolation=cv2.INTER_AREA)
    return _image


def save_pathsList_to_JSON_file(pathsList, file_path="pathsMap_extract.json"):
    json_data = {"paths": pathsList}
    # Save data to JSON file
    # file_path = "pathsMap_extract.json"
    with open(file_path, "w") as outfile:
        json.dump(json_data, outfile)
        # outfile.write(json_data)
        log("pathsList Saved to JSON")


def convert_paths_to_svg(paths, file_path):
    svg_content = '<?xml version="1.0" encoding="UTF-8" standalone="no"?>\n'
    svg_content += '<svg xmlns="http://www.w3.org/2000/svg" version="1.1">\n'

    for path in paths:
        path_string = "M " + " ".join([f"{point[0]},{point[1]}" for point in path])
        svg_content += f'\t<path d="{path_string}" fill="none" stroke="black" />\n'

    svg_content += "</svg>"

    with open(file_path, "w") as svg_file:
        svg_file.write(svg_content)

    log(f"SVG file saved to: {file_path}")


def save_metadata(_final_paths, _non_mirror_final_paths):
    save_pathsList_to_JSON_file(_final_paths)
    save_pathsList_to_JSON_file(
        _non_mirror_final_paths, "non_mirror_pathsMap_extract.json"
    )
    convert_paths_to_svg(_non_mirror_final_paths, "svg_pathsMap_extract.svg")


def isSupportedByView(item):
    for i in SUPPORTED_VIEW_FILE_EXT:
        if i in item.lower():
            return True
    return False


def supported_view_file(list):
    final_list = []

    for p in list:
        if isSupportedByView(p):
            final_list.append(p)
        if "." not in p:
            final_list.append(p)
    return final_list


def get_files(dir_path=""):
    directory_path = dir_path
    if directory_path is "":
        directory_path = os.getcwd()  # current directory
        set_param("current_file_metadata", directory_path)

    # Get a list of files in the directory
    # log(directory_path)
    file_name_list = os.listdir(directory_path)
    # log(str(file_name_list))
    file_name_list = supported_view_file(file_name_list)  # get supported files only

    if file_name_list:
        # Get the full path for each file
        absolute_file_path_list = [
            os.path.join(directory_path, file_name) for file_name in file_name_list
        ]

        # Print the list of files
        # for file_name in absolute_file_path_list:
        # log(file_name)
        return file_name_list, absolute_file_path_list, directory_path
    else:
        log("No supported files")
        return [], [], []


# Get file extension of supported file only
def getFileExt(fileName):
    for ext in SUPPORTED_VIEW_FILE_EXT:
        if ext in fileName.lower():
            return ext
    return None


def isFile(fileName):
    if "." in fileName:
        return True
    return False


# --------------------------------------------------------------------------------------------


def b_spline_path_smoother(waypoints):
    if not waypoints:
        return []

    waypoints = np.array(waypoints)
    x = []
    y = []

    x = waypoints[:, 0]
    y = waypoints[:, 1]

    tck, _ = interpolate.splprep([x, y])
    u = np.linspace(0, 1, num=100)
    _x, _y = interpolate.splev(u, tck)
    smooth = np.dstack((_x, _y))
    # print(smooth)
    smooth = smooth.tolist()
    return smooth[0]


def c_spline_path_smoother(waypoints):
    if not waypoints:
        return []

    waypoints = np.array(waypoints)
    x = []
    y = []

    x = waypoints[:, 0]
    y = waypoints[:, 1]

    tck, _ = interpolate.splprep([x, y], s=1000)
    u = np.linspace(0, 1, num=1000)
    _x, _y = interpolate.splev(u, tck)
    smooth = np.dstack((_x, _y))
    # print(smooth)
    smooth = smooth.tolist()
    return smooth[0]


def smooth_all_paths(paths):
    final = []
    for path in paths:
        smooth = b_spline_path_smoother(path)
        final.append(smooth)
    return final


# Split array into smaller chunks recursivelly (Divide and conquer)
def divco(list, batch_length=NUM_BATCH_LENGTH):
    if len(list) <= batch_length:
        return [list]
    else:
        mid = len(list) // 2
        left = list[:mid]
        right = list[mid:]
        return divco(left) + divco(right)


# Split array into smaller chunks
def divco2(list, chunk_size=50):
    split_arr = [list[i : i + chunk_size] for i in range(0, len(list), chunk_size)]
    return split_arr


def break_down_long_paths(path_list, path_lengh=MAX_THRESHOLD):
    final_paths_list = []
    # For every path break it into chunks and add the chunks in final array
    for path in path_list:
        split_path_list = divco2(path, path_lengh)
        final_paths_list += split_path_list
    return final_paths_list


def init_setup():
    # The default tool frame has it's "z" axis pointing down, for what we are going to do we need it to be pointing up.
    # So we create a new tool frame with a name "tool_frame1" (you can give it any name you want) and
    # rotate it 180degs so that the "z" axis points up.
    set_tool_frame("drawing_tool_frame1", orientation=p[0, 0, 0, 180, 0, 0])
    change_tool_frame("drawing_tool_frame1")  # We make sure we are using it

    # user frame
    try:
        if get_user_frame(USER_FRAME):
            change_user_frame(USER_FRAME)
            movej(p[0, 0, SAFE_HEIGHT, 0, 0, 0])
    except:
        notify(
            'User frame "'
            + str(USER_FRAME)
            + '" Not Found\n'
            + '\nPlease create a user frame called "'
            + str(USER_FRAME)
            + '"',
            error=True,
        )


def start(_paths):
    # notify("Starting")
    try:
        if not _paths:
            notify("No c_paths")
            exit()
        x = 0
        y = 0
        set_path_blending(True, 0)
        for _path in _paths:  # for every path  in c_paths[] list
            try:
                if not len(_path):
                    notify("No c_path")
                    exit()
                movel(Pose(_path[0][0], _path[0][1], 30))
                for point in _path:
                    if point:
                        # log(point)
                        x = point[0]
                        y = point[1]
                        movel(Pose(x, y))
                movel(Pose(x, y, SAFE_HEIGHT))
                # notify("Point: "+str(point), warning=True)
            except Exception as e:
                log("Inner issue:" + str(e))
                # log(_path)
        sync()
        set_path_blending(False)
    except Exception as e:
        log("Something is wrong: " + str(e))
        # log(_paths)


def blend_n_run(batches):
    # pass
    # notify(str(batches[0][0][0]))
    # batch = [[(0,0), (0,100), (0,200), (0, 300)], [(0,0), (0,100), (0,200), (0, 300)]]
    # start(batch) # Go through one batch
    for batch in batches:
        start(batch)


# ***************************
#   Program functions
# ***************************
def render_image_to_preview(img):
    if not len(img):
        log("render_image_to_preview: image issues")
        return
    global temp_image, curr_height, curr_width, image_view

    temp_image = scale_cv_image_WH(img, MIN_WIDTH)
    image_view = temp_image
    curr_height, curr_width, _ = temp_image.shape
    # Convert the image to base64 string
    base64_image = cv_image_to_base64(temp_image)
    set_param("image_view", base64_image)


def handle_files(dir_path=""):
    file_names, absolute_paths, _ = get_files(dir_path)  # Get supported files only
    if len(file_names):
        meta = []
        for i in range(len(file_names)):
            obj = {
                "name": file_names[i],
                "path": absolute_paths[i],
                "isFile": isFile(file_names[i]),
            }
            # log(str(obj))
            meta.append(obj)
        global view_files
        view_files = {"files": meta}
        set_param("view_files", view_files)  # file explorer list
        # for file_name in file_names:
        #    log(file_name)


def handle_path_extraction(t1=INIT_THRESHOLD_1, t2=INIT_THRESHOLD_2):
    set_param("extracting_paths", True)
    img, final_paths, non_mirror_final_paths = extract(
        temp_image, canny_threshold1=t1, canny_threshold2=t2, add_smoothing=get_param("add_smoothing")
    )

    global paths_map, image_view
    image_view = img
    img = cv_image_to_base64(img)
    set_param("image_view", img)
    
    paths_map = final_paths

    # save_metadata(final_paths,non_mirror_final_paths)

    set_param("extracting_paths", False)


def handle_image_scale(w_scale):
    global temp_image, curr_height, curr_width, image_view
    
    temp_image = scale_cv_image_WH(image, w_scale)
    image_view = temp_image
    str_img = cv_image_to_base64(temp_image)
    
    # curr_height, curr_width, _ = temp_image.shape
    set_param("image_view", str_img)

def handle_image_zoom(offset):
    if offset == 0:
        return
    global zoom_scale, image_view
    # height, width, _ = _image.shape
    zoom_scale = zoom_scale + offset
    # log(str(zoom_scale))
     
    str_img = cv_image_to_base64(zoom_image(image_view, zoom_scale))
    
    set_param("image_view", str_img)


def handle_thresholding(t1, t2):
    set_param("extracting_paths", True)
    img, paths_list, _ = extract(temp_image, canny_threshold1=t1, canny_threshold2=t2, add_smoothing=get_param("add_smoothing"))
    img = cv_image_to_base64(img)
    global paths_map
    paths_map = paths_list
    set_param("image_view", img)
    set_param("extracting_paths", False)


def handle_json_file_request(file_path):
    # get path from file
    _paths_map = get_existing_pathsList_JSON(file_path)
    # convert json paths to image
    global image
    image = pathsList_to_cv_image(_paths_map)
    global paths_map
    paths_map = _paths_map
    # render image
    render_image_to_preview(image)


def handle_svg_file_request(file_path):
    set_param("reading_svg_file", True)
    # get path from file
    _paths_map = extract_svg_paths(file_path)
    global image
    image = pathsList_to_cv_image(_paths_map)
    global paths_map
    paths_map = _paths_map
    # render image
    render_image_to_preview(image)
    set_param("reading_svg_file", False)


def handle_folder_request(dir_path):
    handle_files(dir_path)


def handle_current_file_request(file_path):
    if os.path.isdir(file_path):
        # handle folder
        handle_folder_request(file_path)
        set_param("file_view_render_req", True)
    else:
        if ".json" in file_path.lower():
            # handle json
            set_param("image_path_mode", "JSON")  # IMG, JSON, SVG
            handle_json_file_request(file_path)
        elif ".svg" in file_path.lower():
            # handle svg
            set_param("image_path_mode", "SVG")  # IMG, JSON, SVG
            handle_svg_file_request(file_path)
        else:
            # hanlde image
            set_param("image_path_mode", "IMG")  # IMG, JSON, SVG
            preview_file(file_path)
        set_param("is_file_view_visible", False)
        set_param("is_image_preview_visible", True)


def handle_file_path_back_trace(file_path):
    log("Input: " + str(file_path))
    b_traced_path = back_trace_file_path(file_path)
    set_param("current_file_metadata", b_traced_path)
    log("Output: " + str(b_traced_path))

    handle_current_file_request(b_traced_path)


def handle_drawing():
    init_setup()
    if len(paths_map):
        map_list = break_down_long_paths(paths_map, MAX_PATH_LENGTH)
        batches = divco(map_list)
        blend_n_run(batches)
    else:
        notify("No extracted paths found", warning=True)
    set_param("is_drawing", False)


def preview_file(image_path=DEFAULT_IMAGE):
    init_setup()
    global image
    try:
        image = cv2.imread(image_path)
        # log(str(image))
        render_image_to_preview(image)
    except:
        log("Initial Preview: Image access issue")


preview_file()


def request_handler():
    # extraction request
    if get_param("extract_req"):
        set_param("extract_req", False)
        thresh1 = get_param("threshold1_value")
        thresh2 = get_param("threshold2_value")
        # log("extract_req")
        # notify("extract_req")
        # Do something ...
        handle_path_extraction(thresh1, thresh2)

    # import image request
    if get_param("imp_img_req"):
        set_param("imp_img_req", False)
        log("imp_img_req")
        handle_files()
        set_param("file_view_render_req", True)

        set_param("is_file_view_visible", True)
        set_param("is_image_preview_visible", False)
        # notify("imp_img_req")
        # Do something ...

    if get_param("current_file_req"):
        set_param("current_file_req", False)
        file_path = get_param("current_file_metadata")
        log("File Req: " + file_path)
        handle_current_file_request(file_path)

    # import JSON request
    if get_param("imp_json_req"):
        set_param("imp_json_req", False)
        log("imp_json_req")

        # notify("imp_json_req")
        # Do something ...

    # show JSON view request
    if get_param("json_view_req"):
        set_param("json_view_req", False)
        log("json_view_req")
        # notify("json_view_req")
        # Do something ...

    # Image scaling  request (Width : Height)
    if get_param("width_req"):
        set_param("width_req", False)
        slider_value = get_param("width_scale")
        w_scale = int(slider_value)
        # log("SL_V: "+str(slider_value))
        handle_image_scale(w_scale)
        # log("width scale: "+str(w_scale))
        # Do something ...

    # Threshold requests
    if get_param("threshold1_req") or get_param("threshold2_req"):
        thresh1 = get_param("threshold1_value")
        thresh2 = get_param("threshold2_value")
        # NOTE: Don't do path extraction with live thresholding, it will slow down and lag
        # handle_thresholding(thresh1, thresh2)

    if get_param("back_trace_req"):
        set_param("back_trace_req", False)
        log("back_trace_req")
        file_path = get_param("current_file_metadata")
        handle_file_path_back_trace(file_path)

    if get_param("draw_req"):
        set_param("draw_req", False)
        set_param("is_drawing", True)
        handle_drawing()
    if get_param("zoom_req") is not 0:
        offset = get_param("zoom_req")
        set_param("zoom_req", 0)
        # log("Zoom")
        # handle zoom
        handle_image_zoom(offset)


def main():
    # Request variables
    set_param("extract_req", False)
    set_param("imp_img_req", False)
    set_param("imp_json_req", False)
    set_param("json_view_req", False)

    set_param("threshold1_req", False)
    set_param("threshold2_req", False)

    set_param("width_req", False)
    set_param("height_req", False)

    set_param("current_file_req", False)
    set_param("file_view_render_req", True)

    set_param("back_trace_req", False)

    set_param("draw_req", False)

    set_param("zoom_req", 0)

    # variables
    set_param("max_threshold", MAX_THRESHOLD)
    set_param("min_threshold", MIN_THRESHOLD)

    set_param("width_scale", 0)
    set_param("height_scale", 0)

    set_param("min_width", MIN_WIDTH)
    set_param("max_width", MAX_WIDTH)

    set_param("threshold1_value", INIT_THRESHOLD_1)
    set_param("threshold2_value", INIT_THRESHOLD_2)

    set_param("init_threshold1", INIT_THRESHOLD_1)
    set_param("init_threshold2", INIT_THRESHOLD_2)

    set_param("extracting_paths", False)

    set_param("current_file_metadata", "")
    set_param("is_file_view_visible", False)
    set_param("is_image_preview_visible", True)

    set_param("is_drawing", False)
    set_param("e_stop", False)

    set_param("reading_svg_file", False)

    set_param("image_path_mode", "IMG")  # IMG, JSON, SVG

    set_param("add_smoothing", True)
    # vars
    # set_param("image", "")
    # set_param("is_json_view", False)

    while True:
        request_handler()
        sleep(0.2)
