import cv2
import numpy as np

hsv_lower = (48, 116, 28)
hsv_upper = (255, 255, 255)

img = cv2.imread('screenshot.png')
blurred = cv2.GaussianBlur(img, (11, 11), 0)  # apply blur
hsv_frame = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)  # convert from RGB to HSV colorspace
mask = cv2.inRange(hsv_frame, hsv_lower, hsv_upper)  # create a mask over the object to track
mask = cv2.erode(mask, None, iterations=5)
mask = cv2.dilate(mask, None, iterations=5)

contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
print("Number of contours = " + str(len(contours)))

if len(contours) > 0:  # make sure at least 1 contour is obtained
    largest_contour = max(contours, key=cv2.contourArea)  # get the largest contour
    bbox = cv2.minAreaRect(largest_contour)  # create a rectangle using the largest found contour
    bbox = cv2.boxPoints(bbox)
    bbox = np.int0(bbox)

cv2.drawContours(img, contours, 1, (0, 255, 0), 3)
frame = cv2.drawContours(img, [bbox], 0, (0, 255, 0), 3)
# cv2.imwrite('bbox.png', img)

cv2.imshow('Image', img)
cv2.waitKey(0)
cv2.destroyAllWindows()
