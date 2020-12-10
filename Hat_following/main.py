import argparse
import errno
import numpy as npgit
import os
import time
import yaml
from threading import Thread

from djitellopy import Tello

from estimate_tag import PoseEst
from move_drone import MoveDrone


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
offsets_min = config["follow_tag"]["offsets_min"]
offsets_max = config["follow_tag"]["offsets_max"]


# initialize Tello
tello = Tello()
tello.connect()
tello.streamoff()
time.sleep(0.1)
tello.streamon()


pose_estimator = PoseEst(tello)
move_drone = MoveDrone(tello)

while True:
    pose_estimator.update(tello)                    # Reads a single frame from the Tello and then updates the state estimation filter.
    theta = pose_estimator.get_arc_angle()[0]
    _, offsets = pose_estimator.get_target_state()

    if theta_min < theta < theta_max:
        if offsets_min < offsets < offsets_max:
            move_drone.linear_control(0, 0, 0)
        else:
            cmd, speed = linear_controller.tello_track(rvec, tvec)

            pass
            # vx, vy, vz = calculate_xyz_error()
            # go x y z speed

    else:
        pass
        # a, b, c, d = calculate_arc()
        # set rc a b c d
