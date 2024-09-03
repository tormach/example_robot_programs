from robot_command.rpl import *
import rospy
import base64
import json

import os

def log( msg, type="warning"):
    if type == "warning":
        rospy.logwarn(msg)
    if type == "error":
        rospy.logerr(msg)

def get_files(dir_path=""):
    directory_path = dir_path
    if directory_path == "":
        #global directory_path
        directory_path = os.getcwd() # current directory
    
    # Get a list of files in the directory
    #log(directory_path)
    file_name_list = os.listdir(directory_path)
    #log(str(file_name_list))
    #file_name_list = supported_view_file(file_name_list) # get supported files only
    
    if file_name_list:
         #Get the full path for each file
        absolute_file_path_list = [os.path.join(directory_path, file_name) for file_name in file_name_list]

        #Print the list of files
        for file_name in absolute_file_path_list:
            log(file_name)
        return file_name_list, absolute_file_path_list, directory_path
    else:
        log("No supported files")
        return None

#/home/pathpilot/nc_files/robot_programs/demos_sadiq/Drawer_UI_test/assets/
def main():
    get_files("/home/pathpilot/nc_files/robot_programs/demos_sadiq/Drawer_UI_test/assets/")
    exit()

