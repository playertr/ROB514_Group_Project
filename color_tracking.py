#!/usr/bin/env python3
# @author: Khushal Brahmbhatt

"""
Script to track an object of interest in a video stream.

Specify the correct options in the 'config.ini' file before running this.

Usage:
python color_tracking.py

To optionally write the video stream to a file:
python color_tracking.py --save -o filename
"""

import argparse
import configparser
import errno
import os
import time
from collections import deque

import cv2
import numpy as np

# Parse videocapture options
parser = argparse.ArgumentParser(
    description='Specify videocapture options.',
    formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument('--save', default=False, action='store_true', help='Save video stream to file.')
parser.add_argument('-c', '--config', default='config.ini', help='Input config file.')
parser.add_argument('-o', '--output', default=None, help='Specify filename to write to without the extension.')
args = parser.parse_args()

# Load config options
config_file = args.config
try:
    with open(config_file) as f:
        print(f'[INFO] Loading config file: {config_file}')
        config = configparser.ConfigParser()
        config.read_file(f)
except IOError:
    raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), config_file)

# Get video source: camera/video file
try:
    vid_source = config.getint('video', 'source')
except ValueError:
    vid_source = config.get('video', 'source')
buffer = config.getint('color_tracking', 'buffer')  # no. of points to keep track of
erode_iter = config.getint('color_tracking', 'erode_iter')  # no. of iterations of erosion to apply for preprocessing
dilate_iter = config.getint('color_tracking', 'dilate_iter')  # no. of iterations of dilation to apply for preprocessing
blur_kernel = config.get('color_tracking', 'blur_kernel')  # kernel to use for blurring the image for preprocessing
hsv_lower = config.get('color_tracking', 'hsv_lower')  # lower hsv limits of object to track
hsv_upper = config.get('color_tracking', 'hsv_upper')  # upper hsv limits of object to track
# convert strings to tuples
blur_kernel = tuple(map(int, blur_kernel.split(',')))
hsv_lower = tuple(map(int, hsv_lower.split(',')))
hsv_upper = tuple(map(int, hsv_upper.split(',')))

tracked_pts = deque(maxlen=buffer)  # double-ended queue for storing tracked points

# Load file write options if '--save' option is specified
if args.save:
    if args.output and os.path.isdir(os.path.dirname(args.output)):
        output_file = args.output
        print(f'[INFO] Video will be saved to {output_file}.avi')
    else:
        output_file = 'autosave'
        print(f'[INFO] Filename not given or wrong filepath. Video will be saved to {output_file}.avi')

    fps = config.getint('video', 'fps')
    width = config.getint('video', 'width')
    height = config.getint('video', 'height')
    codec = config.get('video', 'codec')  # codec to use for writing the video
    ext = config.get('video', 'extension')  # file extension

    vid_out = f'{output_file}{ext}'  # file to write video to
    fourcc = cv2.VideoWriter_fourcc(*f'{codec}')
    writer = cv2.VideoWriter(vid_out, fourcc, fps, (width, height))  # intialize video writer

# Start video stream
stream = cv2.VideoCapture(vid_source)
time.sleep(2)  # allow some time to grab the video stream

# keep looping while the stream is on
while stream.isOpened():
    grabbed, frame = stream.read()  # read the video stream and grab the current frame
    blurred = cv2.GaussianBlur(frame, blur_kernel, 0)  # apply blur
    hsv_frame = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)  # convert from RGB to HSV colorspace

    mask = cv2.inRange(hsv_frame, hsv_lower, hsv_upper)  # create a mask over the object to track
    mask = cv2.erode(mask, None, iterations=erode_iter)  # erode to remove small blobs and edges
    mask = cv2.dilate(mask, None, iterations=dilate_iter)  # dilate to get smooth edges

    # Find contours in the mask
    contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    center = None  # center of detected object
    # Draw bounding box
    if len(contours) > 0:  # make sure at least 1 contour is obtained
        largest_contour = max(contours, key=cv2.contourArea)  # get the largest contour
        bbox = cv2.minAreaRect(largest_contour)  # create a rectangle using the largest found contour
        bbox = cv2.boxPoints(bbox)
        bbox = np.int0(bbox)
        center = tuple(bbox.mean(axis=0).astype('int'))  # get the center of the rectangle

        # draw the bounding box around the detected object
        frame = cv2.drawContours(frame, [bbox], 0, (0, 255, 0), 3)

    # Update tracked points using center of rectangle
    tracked_pts.appendleft(center)

    # Draw tracker
    # loop over the tracked points
    for i in range(1, len(tracked_pts)):
        # ignore if either of the tracked points are None
        if tracked_pts[i - 1] is None or tracked_pts[i] is None:
            continue

        thickness = int(np.sqrt(buffer / float(i + 1)) * 2.5)  # thickness of tracking line

        # Draw the tracked line
        cv2.line(frame, tracked_pts[i - 1], tracked_pts[i], (0, 0, 255), thickness)

    # Write frame to video file if '--save' option is specified
    if args.save:
        writer.write(frame)

    # Show the current frame
    cv2.imshow('frame', frame)

    # Stop streaming the video if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

stream.release()  # release the video stream
cv2.destroyAllWindows()  # close all open windows
