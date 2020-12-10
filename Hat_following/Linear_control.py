#!/usr/bin/env python

# ROB 514 Project
#
# linear_control.py
#
# Shital Sable
#
# This script controls the linear motion of the drone and makes sure that the robot stays in the center of the frame.

from estimate_tag import PoseEst, SimpleFilter
import time
import cv2
import datetime
import numpy as np

 #Python library that allows you to create multiple threads to run multiple functions at the same time. 

def main():
    """ Create a tello controller and show the video feed."""
    tellotrack = TelloControl()
    while True:
        tellotrack.tello_track()


class TelloControl:
    """
    This class has a script to control the linear movements of the drone to make sure it
    is in the center of the frame
    """
    def __init__(self):
        self.pose = PoseEst()
        self.init_PID()

    def init_PID(self):
        def proportional():
            zoff_thresh = 1			#currently set as 1 m distance from the target
            Vx = 0
            Vy = 0
            Vz = 0
            prev_time = time.time()
            Ix = 0
            Iy = 0
            Iz = 0
            xoff_prev = 0
            yoff_prev = 0
            zoff_prev = 0
            while True:
                #yield an x, y and z velocity from xoff, yoff, zoff
                xoff, yoff, zoff = yield Vx, Vy, Vz
                zoff = zoff - zoff_thresh   #distance to maintain from the target

                #PID Calculations
                current_time = time.time()
                delta_t = current_time - prev_time

                #Control Equations, constants are adjusted as needed
                Px = 0.1*xoff
                Py = 0.1*yoff
                Pz = 0.1*zoff

                Ix = Ix + -0.001*xoff*delta_t
                Iy = Iy + -0.001*yoff*delta_t
                Iz = Iz + -0.001*zoff*delta_t

                Dx = 0.01*(xoff - xoff_prev)/(delta_t)
                Dy = 0.01*(yoff - yoff_prev)/(delta_t)
                Dz = 0.01*(zoff - zoff_prev)/(delta_t)

                Vx = Px + Ix + Dx
                Vy = Py + Iy + Dy
                Vz = Pz + Iz + Dz

                #update the stored data for the next iteration
                xoff_prev = xoff
                yoff_prev = yoff
                zoff_prev = zoff

                prev_time = current_time
        self.PID = proportional()
        self.PID.send(None)

    def tello_track(self):
        """convert frame to cv2 image and show"""
        self.pose.update()
        self.pose.draw_estimate()
        _, tvec = self.pose.read_image_pose()  # input translational vector here
        if tvec != None:
            xoff, yoff, zoff = [0.001, 0.001, 0.001]
        else:
            xoff, yoff, zoff = [elem for elem in tvec]

        Vx,Vy, Vz = self.PID.send([xoff, yoff, zoff])


        # Create a loop to implement the Vx, Vy and Vz as a command to move the drone accordingly
        cmd = ""
        speed = 0
       # if self.tracking:
        if abs(Vx) > abs(Vy) or abs(Vz):
            if Vx < 0:
                cmd = "right"
                speed = abs(Vx)
            elif Vx > 0:
                cmd = "left"
                speed = abs(Vx)
        elif abs(Vy) > abs(Vx) or abs(Vz):
            if Vy < 0:
                cmd = "up"
                speed = abs(Vy)
            elif Vy > 0:
                cmd = "down"
                speed = abs(Vy)

        else:
            if Vz < 0:
                cmd = "forward"
                speed = abs(Vz)
            elif Vz > 0:
                cmd = "backward"
                speed = abs(Vz)
        print(cmd)
        print(speed)
        return cmd, speed

if __name__ == '__main__':
    main()
