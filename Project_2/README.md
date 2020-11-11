# ROB514_Group_Project

## Usage
1. Update calibration images for the Tello by placing it under /calib_images/checkerboard, saved as .jpg files. Example calibration images are currently under aforementioned folder.

2. Track ArUCo markers with `aruco_tracker.py`:
    ```
    python3 aruco_tracker.py
    ```
    Make sure the computer is connected to the Tello's Wi-Fi network prior to running this script
