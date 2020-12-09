# https://longervision.github.io/2017/03/13/ComputerVision/OpenCV/opencv-external-posture-estimation-ChArUco-board/
import numpy as np
import cv2
import cv2.aruco as aruco
import glob
from image_grab import Record

aruco_dict = aruco.Dictionary_get( aruco.DICT_6X6_1000 )

squareLength = 40   # Here, our measurement unit is centimetre.
markerLength = 30   # Here, our measurement unit is centimetre.
board = aruco.CharucoBoard_create(5, 7, squareLength, markerLength, aruco_dict)

arucoParams = aruco.DetectorParameters_create()

# videoFile = "charuco_board_57.mp4"
# cap = cv2.VideoCapture(videoFile)
cap = cv2.VideoCapture(0)

while(True):
    ret, frame = cap.read() # Capture frame-by-frame
    if ret == True:
        # frame_remapped = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR, cv2.BORDER_CONSTANT)    # for fisheye remapping
        frame_remapped = frame
        frame_remapped_gray = cv2.cvtColor(frame_remapped, cv2.COLOR_BGR2GRAY)

        corners, ids, rejectedImgPoints = aruco.detectMarkers(frame_remapped_gray, aruco_dict, parameters=arucoParams)  # First, detect markers
        aruco.refineDetectedMarkers(frame_remapped_gray, board, corners, ids, rejectedImgPoints)
        im_with_charuco_board = frame
        if ids != None: # if there is at least one marker detected
            charucoretval, charucoCorners, charucoIds = aruco.interpolateCornersCharuco(corners, ids, frame_remapped_gray, board)
            im_with_charuco_board = aruco.drawDetectedCornersCharuco(frame_remapped, charucoCorners, charucoIds, (0,255,0))
            retval, rvec, tvec = aruco.estimatePoseCharucoBoard(charucoCorners, charucoIds, board, camera_matrix, dist_coeffs)  # posture estimation from a charuco board
            if retval == True:
                im_with_charuco_board = aruco.drawAxis(im_with_charuco_board, camera_matrix, dist_coeffs, rvec, tvec, 100)  # axis length 100 can be changed according to your requirement
        else:
            im_with_charuco_left = frame_remapped

        cv2.imshow("charucoboard", im_with_charuco_board)

        if cv2.waitKey(2) & 0xFF == ord('q'):
            break
    else:
        break

cap.release()   # When everything done, release the capture
cv2.destroyAllWindows()