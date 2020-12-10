import argparse
import csv
import errno
import numpy as np
import os
import time
import yaml

from djitellopy import Tello
import keyboard

from estimate_tag import PoseEst
from move_drone import MoveDrone

LOGGING_INTERVAL = 0.1

# parse args
parser = argparse.ArgumentParser(
    description='Specify file and videocapture options.',
    formatter_class=argparse.ArgumentDefaultsHelpFormatter)
# parser.add_argument('--save', default=False, action='store_true', help='Save video stream to file.')
parser.add_argument('-c', '--config', default='config.yaml', help='Input config file.')
# parser.add_argument('-o', '--output', default=None, help='Specify filename to write to without the extension.')
args = parser.parse_args()


# load params from config.yaml
config_file = args.config
try:
    with open(config_file) as f:
        print(f'[INFO] Loading config file: {config_file}')
        config = yaml.load(f)
except IOError:
    raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), config_file)

theta_min = config["follow_tag"]["theta_min"] * np.pi / 180
theta_max = config["follow_tag"]["theta_max"] * np.pi / 180
offsets_min = np.array(config["follow_tag"]["offsets_min"])
offsets_max = np.array(config["follow_tag"]["offsets_max"])
k_constant = config["follow_tag"]["k_constant"]


# initialize Tello
tello = Tello()
tello.connect()
tello.streamoff()
time.sleep(0.1)
tello.streamon()

pose_estimator = PoseEst()
move_drone = MoveDrone(tello)

move_drone.takeoff()


# Create new log file
# i=0
# while os.path.exists("log%s.csv" % i):
#     i += 1
# with open("log%s.csv" % i, "w") as csvfile:
#     writer = csv.writer(csvfile)
#     old_time = time.time()


while True:
    if keyboard.is_pressed('ctrl+q'):
        move_drone.land()
        break

    if keyboard.is_pressed('ctrl+e'):
        move_drone.emergency()
        break

    pose_estimator.update(tello)  # Reads a single frame from the Tello and then updates the state estimation filter.
    _, offsets = pose_estimator.get_target_state()
    vx, vy, vz = (0, 0, 0)

    if offsets is None:
        move_drone.linear_control(vx, vy, vz)
        continue

    offsets = offsets.flatten()
    theta = pose_estimator.get_arc_angle()[0]
    print('Offsets: {}, Theta: {}'.format(offsets, theta))

    if theta_min < theta < theta_max:
        if (offsets_min < offsets).all() and (offsets < offsets_max).all():
            move_drone.linear_control(vx, vy, vz)
        else:
            vx, vy, vz = move_drone.calc_linear_vel(offsets, offsets_min, offsets_max, k_constant)
            print('Vx: {}, Vy: {}, Vz: {}'.format(vx, vy, vz))
            move_drone.linear_control(vx, -vy, vz)
    else:
        if theta < theta_min:
            move_drone.arc_cw()
        elif theta > theta_max:
            move_drone.arc_ccw()

    # if time.time() - old_time > LOGGING_INTERVAL:
    #     writer.writerow([time.time(), offsets, vx, vy, vz])
    #     old_time = time.time()
