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
    videoCaptureObject = cv2.VideoCapture(0)
    result = True
    while (result):
        ret, frame = videoCaptureObject.read()
        cv2.imwrite(filename, frame)
        result = False
    videoCaptureObject.release()
    cv2.destroyAllWindows()