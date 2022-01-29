from robot_command.rpl import *
import os, datetime

try:
    import cv2
except:
    from sh import pip3
    with sh.contrib.sudo:
        pip3.install("opencv-python")
    import cv2

set_units("mm", "deg")

def main():
    # get the current working directory (so we know where the jpg file is saved)
    cwd = os.getcwd()
    # let's name the jpg 'test_photo' and add a timestamp
    filename = "test_photo" + str(datetime.datetime.now()) + ".jpg"
    # take the photo
    take_snapshot(filename)
    # give the notify popup the full path to the file, relative paths not working as of 10.13.21
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
        cv2.imwrite(filename, frame)
        result = False
    videoCaptureObject.release()
    cv2.destroyAllWindows()