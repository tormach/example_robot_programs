from robot_command.rpl import *

set_units("mm", "deg")
import os, datetime

try:
    import cv2
except:
    from sh import pip3
    with sh.contrib.sudo:
        pip3.install("opencv-python")
    import cv2

def main():
    cwd = os.getcwd()
    filename = "test_photo" + str(datetime.datetime.now()) + ".jpg"
    take_snapshot(filename)
    filepath = os.path.join(cwd, filename)
    notify("Here is your photo: ", warning=True, image_path=filepath)

def take_snapshot(filename):
    # select the video device index (usually '0', but can be higher number)
    camera_index = 0
    # open the connection to camera and create control object
    videoCaptureObject = cv2.VideoCapture(camera_index)
    # check we are connected and hard fail if not
    if not videoCaptureObject.isOpened():
        notify(f"Cannot access the camera on index '{camera_index}'. "
               f"Please, check the /dev/video{camera_index} device.", error=True)
    result = True
    while (result):
        ret, frame = videoCaptureObject.read()
        # throw away first frame because some cameras return a 'blank' frame for the first read
        ret, frame = videoCaptureObject.read()
        cv2.imwrite(filename, frame)
        result = False
    videoCaptureObject.release()
    cv2.destroyAllWindows()
