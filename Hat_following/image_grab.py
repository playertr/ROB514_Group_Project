#!/usr/bin/env python3

# Stream live video from the Tello drone and view it on the computer
#
# Farhan Rozaidi

# Note: Turn Tello on and connect to its Wi-Fi network prior to running this script

# Import OpenCV and Tello packages
import cv2, time
from djitellopy import Tello

# Store the commands in a class to make it more easily integratable with movement
# commands and OpenCV processing later on
class Record:
    def __init__(self):
        self.tello = Tello()
        # Attempt to establish the connection betweeen the Tello and computer
        self.tello.connect()
        # Turn off the stream precautionarily
        self.tello.streamoff()
        time.sleep(0.1)
        # Turn on the stream
        self.tello.streamon()
    # Define a function to start the video stream
    def video_stream(self):
        # Read the current frame from the Tello
        read_frame = self.tello.get_frame_read()
        cur_frame = read_frame.frame
        # Obtain the height and width of the frame
        height, width, _ = read_frame.frame.shape
        # Resize the frame within OpenCV
        img = cv2.resize(cur_frame,(width,height))
        return img
