"""
estimate_tag.py
Pose estimation for ArUCo tag

Tim Player
8 December 2020
playert@oregonstate.edu
Modified from Aruco_Tracker project, inherited MIT license from that.

References  :
    1) https://docs.opencv.org/3.4.0/d5/dae/tutorial_aruco_detection.html
    2) https://docs.opencv.org/3.4.3/dc/dbb/tutorial_py_calibration.html
    3) https://docs.opencv.org/3.1.0/d5/dae/tutorial_aruco_detection.html
"""

import numpy as np
import cv2
import cv2.aruco as aruco
import glob
import queue

USE_WEBCAM = True # use webcam instead of drone stream (for debugging)
MAX_TIMES_WITHOUT_READING = 20

class SimpleFilter:
    """ Filter which simply returns the last known value.
    In the future, this could be changed to a legit Kalman filter
    or an ill-legit exponential weighted moving average filter. 
    
    If no value is given for more than MAX_TIMES_WITHOUT_READING, 
    the filter resets.
    """

    def __init__(self):
        self.reset()
        
    def reset(self):
        self.current_val = None
        self.num_times_without_reading = 0

    def update_filter(self, val):
        """ Updates the `current_val` field. `val` can have any type. """
        if val is not None:
            self.current_val = val
            self.num_times_without_reading = 0
        else: 
            self.num_times_without_reading += 1
        
        if self.num_times_without_reading > MAX_TIMES_WITHOUT_READING:
            self.reset()

    def get_current_val(self):
        return self.current_val

class PoseEst:
    """ Class that maintains an estimate of the marker position relative to drone. """
    
    def __init__(self):
        """ Create queue of last-known positions and start the camera. """

        # Calibrate camera (always necessary because it gives mtx and dist)
        ret, mtx, dist, rvecs, tvecs = self.get_calibration()
        self.mtx  = mtx
        self.dist = dist

        # Start the drone stream
        if not USE_WEBCAM:
            self.drone = Record()
            # TODO: import Record
        else:
            self.cap = cv2.VideoCapture(0)

        self.draw_est_info = (None, None, None) # Tuple containing objects necessary to draw the estimate
        self.rvecs_filter = SimpleFilter()
        self.tvecs_filter = SimpleFilter()   
 
    def update(self):
        """ Updates the state estimate using new image. """
        rvecs, tvecs = self.read_image_pose()     # Get new pose estimate from an image
        self.rvecs_filter.update_filter(rvecs)    # Update rvecs filter
        self.tvecs_filter.update_filter(tvecs)    # Update tvecs filter

    def get_target_state(self):
        """ Return rvecs, tvecs for orientation and position of marker. """
        # TODO: add note regarding frame convention

        rvecs = self.rvecs_filter.get_current_val()
        tvecs = self.tvecs_filter.get_current_val()

        return rvecs, tvecs

    def read_image_pose(self):
        """ Query the drone for a single image and estimate the pose rvecs and tvecs. """

        if USE_WEBCAM:
            ret, frame = self.cap.read()
        else:
            frame = self.drone.video_stream()
    
        # operations on the frame
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # set dictionary size depending on the aruco marker selected
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

        # detector parameters can be set here (List of detection parameters[3])
        parameters = aruco.DetectorParameters_create()
        parameters.adaptiveThreshConstant = 10

        # lists of ids and the corners belonging to each id
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        rvecs, tvecs = None, None

        # check if the ids list is not empty
        # if no check is added the code will crash
        if np.all(ids != None):

            # estimate pose of each marker and return the values
            # rvet and tvecs-different from camera coefficients
            rvecs, tvecs ,_ = aruco.estimatePoseSingleMarkers(corners, 0.05, self.mtx, self.dist)

        self.draw_est_info = (ids, corners, frame)

        return rvecs, tvecs

    def draw_estimate(self):
        """ Draw the current estimate to the screen. """

        rvecs, tvecs = self.get_target_state()
        ids, corners, frame = self.draw_est_info
        
        # font for displaying text
        font = cv2.FONT_HERSHEY_SIMPLEX

        if rvecs is not None:
                for i in range(rvecs.shape[1]):
                    # draw axis for the aruco markers
                    aruco.drawAxis(frame, self.mtx, self.dist, rvecs[i], tvecs[i], 0.1)

                # draw a square around the markers
                aruco.drawDetectedMarkers(frame, corners)

                # cv2.putText(frame, "Id: " + strg, (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)
                cv2.putText(frame, f"Translation:{tvecs}", (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)

        else:
            # code to show 'No Ids' when no markers are found
            cv2.putText(frame, "No Ids", (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)

        # display the resulting frame
        cv2.imshow('frame',frame)

    def get_calibration(self):
        # termination criteria for the iterative algorithm
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        # checkerboard of size (7 x 6) is used
        # objp = np.zeros((6*7,3), np.float32)
        # objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)

        # Checkerboard size
        rows=7
        cols=10
        objp = np.zeros((cols*rows,3), np.float32)
        objp[:,:2] = np.mgrid[0:rows,0:cols].T.reshape(-1,2)

        # arrays to store object points and image points from all the images.
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.

        # iterating through all calibration images
        # in the folder
        images = glob.glob('calib_images/checkerboard2/*.JPG')
        count = 0
        for fname in images:
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

            # find the chess board (calibration pattern) corners
            # ret, corners = cv2.findChessboardCorners(gray, (7,6),None)
            ret, corners = cv2.findChessboardCorners(gray, (rows,cols),None)
            if ret:
                print(f"Found corners for image {count}")
            else:
                print(f"No corners found for image {count}")
            count += 1

            # if calibration pattern is found, add object points,
            # image points (after refining them)
            if ret == True:
                objpoints.append(objp)

                # Refine the corners of the detected corners
                corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
                imgpoints.append(corners2)

                # Draw and display the corners
                img = cv2.drawChessboardCorners(img, (rows,cols), corners2,ret)

        return cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

# Run as a script if this file is called.
if __name__ == "__main__":
    pe = PoseEst()

    while(True):
        pe.update()
        pe.draw_estimate()
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break