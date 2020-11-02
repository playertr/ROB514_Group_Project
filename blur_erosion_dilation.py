
#!/usr/bin/env python3
# @author: Tim Player

"""
Script to demonstrate Gaussian blur, erosion, and dilation. Displays and saves files blur.jpg, erosion.jpg, dilation.jpg.

Usage:
python blur_erosion_dilation.py.
"""
import cv2
import numpy as np

# HSV boundaries for mask, detecting red
HSV_LOWER = (48, 116, 28)
HSV_UPPER = (255, 255, 255)

def main():
    # Open image of Farhan with hat on
    image = cv2.imread('farhan.png')
    display(image, "0_Farhan")

    # Apply blur
    blurred = cv2.GaussianBlur(image, (11, 11), 0)
    display(blurred, "1_Blurred")

    # Convert from RGB to HSV colorspace and create a mask over the hat
    hsv_frame = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)  
    mask = cv2.inRange(hsv_frame, HSV_LOWER, HSV_UPPER) 
    display(mask, "2_Initial_Mask")

    # Erode mask to remove small blobs and edges
    mask = cv2.erode(mask, None, iterations=5)
    display(mask, "3_Eroded_Mask")

    # Dilate mask to get smooth edges
    mask = cv2.dilate(mask, None, iterations=5)
    display(mask, "4_Dilated_Mask")

def display(image, name="Image"):
    """ Display an image, save it, then DESTROY ALL WINDOWS to prevent hanging. """
    cv2.imshow(name, image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    cv2.imwrite("{}.png".format(name), image)

if __name__ == '__main__':
    main()