# ROB514_Group_Project

## Usage
1. Get HSV limits using `range_detector.py`:
    ```
    python range-detector.py --webcam --filter HSV
    ```
    Make sure the object you want to track is white i.e. unmasked. The background should be black i.e. masked.

2. Update the HSV limits and set your camera device or video file as the source in `config.ini`.

2. Track object using `color_tracking.py`:
    ```
    python color_tracking.py -c config.ini
    ```
   Default config file is config.ini
   
   If you want to save the video stream to a file:
   ```
   python color_tracking.py --save -o filename
   ```
    Specify filename without the extension.
    
    Default save location is autosave.avi