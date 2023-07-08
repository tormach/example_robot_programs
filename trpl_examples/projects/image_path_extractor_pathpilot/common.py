import numpy as np
import cv2
import json

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

def save_pathsList_to_JSON_file(pathsList, file_path="pathsMap_extract.json"):
    json_data = {
        "paths": pathsList
    }
    # Save data to JSON file
    # file_path = "pathsMap_extract.json"
    with open(file_path, "w") as outfile:
        json.dump(json_data, outfile)
        # outfile.write(json_data)
        print("pathsList Saved to JSON")

def cv_strict_resize(img, _desired_width=150, _desired_height=150):
    # Calculate the aspect ratio of the image
    aspect_ratio = img.shape[1] / img.shape[0]
    if (img.shape[1] > img.shape[0]):
        aspect_ratio = img.shape[0]/img.shape[1]
    print("W:H", img.shape[1], img.shape[0])
    print("aspect_ratio :"+ str(aspect_ratio))
    # Calculate the new dimensions based on the aspect ratio
    if _desired_width / aspect_ratio <= _desired_height:
        new_width = _desired_width
        new_height = int(new_width / aspect_ratio)
    else:
        new_height = _desired_height
        new_width = int(new_height * aspect_ratio)
    print("new W:H", new_width, new_height)
    return cv2.resize(img, (new_width, new_height), interpolation=cv2.INTER_AREA)