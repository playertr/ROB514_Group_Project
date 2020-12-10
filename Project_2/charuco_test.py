# https://longervision.github.io/2017/03/13/ComputerVision/OpenCV/opencv-external-posture-estimation-ChArUco-board/
import numpy as np
import cv2
import cv2.aruco as aruco
import glob
from image_grab import Record
import pdb
import matplotlib.pyplot as plt 
import matplotlib as mpl

aruco_dict = aruco.Dictionary_get( aruco.DICT_6X6_1000 )

squareLength = 40   # Here, our measurement unit is centimetre.
markerLength = 30   # Here, our measurement unit is centimetre.
board = aruco.CharucoBoard_create(5, 7, squareLength, markerLength, aruco_dict)

arucoParams = aruco.DetectorParameters_create()

# videoFile = "charuco_board_57.mp4"
# cap = cv2.VideoCapture(videoFile)
cap = cv2.VideoCapture(0)

####---------------------- CALIBRATION ---------------------------
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
# images = glob.glob('calib_images/checkerboard/*.jpg')
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

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)


while(True):
    ret, frame = cap.read() # Capture frame-by-frame
    if ret == True:
        # frame_remapped = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR, cv2.BORDER_CONSTANT)    # for fisheye remapping
        frame_remapped = frame
        frame_remapped_gray = cv2.cvtColor(frame_remapped, cv2.COLOR_BGR2GRAY)

        corners, ids, rejectedImgPoints = aruco.detectMarkers(frame_remapped_gray, aruco_dict, parameters=arucoParams)  # First, detect markers
        aruco.refineDetectedMarkers(frame_remapped_gray, board, corners, ids, rejectedImgPoints)
        im_with_charuco_board = frame
        if ids is not None: # if there is at least one marker detected
            charucoretval, charucoCorners, charucoIds = aruco.interpolateCornersCharuco(corners, ids, frame_remapped_gray, board)
            im_with_charuco_board = aruco.drawDetectedCornersCharuco(frame_remapped, charucoCorners, charucoIds, (0,255,0))
            retval, rvec, tvec = aruco.estimatePoseCharucoBoard(charucoCorners, charucoIds, board, mtx, dist, rvec=None, tvec=None)  # posture estimation from a charuco board
            if retval == True:
                im_with_charuco_board = aruco.drawAxis(im_with_charuco_board, mtx, dist, rvec, tvec, 100)  # axis length 100 can be changed according to your requirement
        else:
            im_with_charuco_left = frame_remapped

        cv2.imshow("charucoboard", im_with_charuco_board)

        if cv2.waitKey(2) & 0xFF == ord('q'):
            break
    else:
        break

cap.release()   # When everything done, release the capture
cv2.destroyAllWindows()