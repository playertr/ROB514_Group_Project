#!/usr/bin/env python3

# Move tello based on velocity commands
#
# Farhan Rozaidi

# Note: Turn Tello on and connect to its Wi-Fi network prior to running this script

# Import Tello package
from djitellopy import Tello
from math import pi,sin,cos
import time

class MoveDrone:
    def __init__(self,tello):
        self.tello = tello

    #The first few commands below are fairly straightforward, doesn't need much
    #explanation

    def takeoff(self):
        self.tello.takeoff()

    def land(self):
        self.tello.land()

    def emergency(self):
        self.tello.emergency()

    # This command uses the output of the linear control script to give rc
    # commands to the drone without any yaw
    def linear_control(self,Vx,Vy,Vz):
        self.tello.send_rc_control(Vx,Vy,Vz,0)

    # This function calculates the x,y coordinate pairs in a circle
    # Used for arc calculations later
    def points_in_circle(self,radius,velocity):

        points = []

        angle_iteration = 1
        radian = (angle_iteration*pi)/180

        x_0 = radius
        y_0 = 0
        points.append((x_0,y_0))

        time_step = (2.0*float(radius)/float(velocity))*sin(float(radian)/2.0)

        for i in range(360):
            point = i+1
            x_i = radius*cos(point*radian)
            y_i = radius*sin(point*radian)
            points.append((round(x_i),round(y_i)))

        return points,time_step

    # This function uses the circle function to calculae a portion of the
    # circle in an arc
    def points_in_arc(self,radius,velocity,angle):
        points,time_step = self.points_in_circle(radius, velocity)

        arc_points = []

        for i in range(angle):
            position = i

            arc_points.append(points[position])
        return arc_points,time_step

    # Using the arc points function, this allows the drone to move in an arc
    # with a given radius and angle
    ### note: yaw control has to be set experimentally, still working on it ###
    def arc_control(self,points,time_step):
        for i in points:
            x = i[0]
            y = i[1]
            # rc control here is rough, still need to figure out proper yaw control
            self.tello.send_rc_control(x,y,0,10)
            time.sleep(time_step)
        self.tello.stop()

    def arc_cw(self):
        self.tello.send_rc_control(-40,0,0,40)

    def arc_ccw(self):
        self.tello.send_rc_control(40,0,0,-40)

    def stop(self):
        self.tello.send_rc_control(0,0,0,0)

    def calc_linear_vel(self, tvec, zoff_thresh, k):
        """convert frame to cv2 image and show"""
        if tvec is None:
            xoff, yoff, zoff = [0, 0, 0]  # [0.001, 0.001, 0.001]
        else:
            xoff, yoff, zoff = [elem for elem in tvec.flatten()]

        zoff = zoff - zoff_thresh  # distance to maintain from the target

        # Control Equations, constants are adjusted as needed
        Vx = k[0] * xoff
        Vy = k[1] * yoff
        Vz = k[2] * zoff

        return Vx, Vy, Vz
