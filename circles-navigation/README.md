# ROV Autonomous Navigation and Circle Detection


## Overview
This project involves the development of a system for autonomously navigating a Remotely Operated Vehicle (ROV) through a series of circles using real-time image processing and ROS 2. The system includes real-time circle detection, autonomous control, and a GUI for setting orientations and confirming detected circles. Each circle has a diameter of 80 cm and is positioned 10 cm above the ground. The goal is to successfully navigate each circle without touching it. 

## Table of Contents

- [How it Works?](#how-it-works)
- [Scripts](#scripts)
- [Requirements](#requirements)

___

## How it Works?

Our approach to achieving autonomous navigation through the circles involves several key steps:

1. **Real-time Circle Detection**:
   - Capturing frames from the ROV's camera.
   - Detecting circles in the frames using the Hough Circle Transform method.
   - Displaying the detected circles on the frame.

2. **Autonomous Control**:
   - Subscribing to the ROV's depth information.
   - Using the circle detection algorithm to identify circle positions.
   - Calculating the required movement directions (dx, dy, dz) to navigate through the detected circles.
   - Publishing velocity commands to the ROV based on the detected circles and the calculated movement directions.
   - Adjusting the ROV's path based on feedback from the circle detector to ensure it passes through each circle correctly.

3. **GUI for Orientation and Confirmation**:
   - Allowing the pilot to set the orientations for the circles using radio buttons. The orientation indicates the position of each circle relative to the previous one. For example, {"right", "left"} means that the second circle is to the right of the first circle, and the third circle is to the left of the second circle.
   - Displaying the current orientations and allowing the pilot to confirm the detected circles.
   - If confirmed, the system proceeds with autonomous control using the selected orientations.


___
## Scripts
1. **`src/detect/real_time_circles_detection.py`**:
   - Captures frames from the camera.
   - Detects circles using the CirclesDetector class.
   - Displays the detected circles on the frame.
  
2. **`src/detect/circles_detector.py`**:
   - Contains the CirclesDetector class used for detecting circles in frames.
  
3. **`src/detect/color_correct.py`**:
   - Performs color correction on images and videos.

4. **`src/control/autonomous_control.py`**:
   - Implements the autonomous control logic for the ROV using ROS 2.
   - It subscribes to depth information, detects circles, calculates movement directions, and publishes velocity commands.

5. **`src/control/orientations_and_confirmation.py`**:
   - Provides a GUI for setting circle orientations and confirming detections.
   - If the user confirms the detected circles, it triggers the autonomous control script.
  
___
### Requirements

Before running this project, ensure you have the following dependencies installed:

- **Python 3.7+**
- **Numpy**
- **OpenCV**: This is used for real-time video capture, image processing, and circle detection.
- **Tkinter**: Required for the GUI interface to set circle orientations using radio buttons.
- **rclpy**: ROS 2 client library for Python, used for communication with the ROV.
- **ROS 2 Foxy**: Framework for robotic systems development, necessary for ROS 2 operations.
- **Multiprocessing**: Used for concurrent execution of processes in Python.
