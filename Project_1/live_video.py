#!/usr/bin/env python3

# Stream live video from the Tello drone and view it on the computer
#
# Farhan Rozaidi

# Note: Turn Tello on and connect to its Wi-Fi network prior to running this script

# Import OpenCV and Tello packages
import cv2
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
    # Define a function to start the video stream
    def video_stream(self):
        # Turn on the stream
        self.tello.streamon()
        # Loop within the function to constantly update the frame and publish
        # each frame to the computer
        while True:
            # Read the current frame from the Tello
            read_frame = self.tello.get_frame_read()
            cur_frame = read_frame.frame
            # Obtain the height and width of the frame
            height, width, _ = read_frame.frame.shape
            # Resize the frame within OpenCV
            img = cv2.resize(cur_frame,(width,height))
            # Show the stream on the computer
            cv2.imshow('Stream',img)
            # While the stream is on, if 'q' is pressed, it will turn off the stream
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break


if __name__ == '__main__':
    # Initialize the class
    record = Record()
    # Call the video streaming function
    record.video_stream()
