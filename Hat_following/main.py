import time
from .estimate_tag import PoseEst
from djitellopy import Tello

# argparse if required

# load params from config.yaml
rvec_min = [0,0,0]
rvec_max = [0,0,0]
tvec_min = [0,0,0]
tvec_max = [0,0,0]


# connect to Tello
tello = Tello
tello.connect()
tello.streamoff()
time.sleep(1)
tello.streamon()



state_estimator = PoseEst(tello)

while True: # or while key to abort not pressed
    rvec, tvec = state_estimator.read_image_pose()

    if rvec_min < rvec < rvec_max:
        if tvec_min < tvec < tvec_max:
            continue
        else:
            pass
            # x, y, z = calculate_xyz_error()
            # go x y z speed
    else:
        pass
        # a, b, c, d = calculate_arc()
        # set rc a b c d
