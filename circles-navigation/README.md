# ROV Autonomous Navigation and Circle Detection

This project involves the development of a system for autonomously navigating a Remotely Operated Vehicle (ROV) through a series of circles using real-time image processing and ROS 2. The system includes real-time circle detection, autonomous control, and a GUI for setting orientations and confirming detected circles. Each circle has a diameter of 80 cm and is positioned 10 cm above the ground. The goal is to successfully navigate each circle without rotating the ROV. 

## Table of Contents

- [Project Structure](#project-structure)
  - [real_time_circles_detection.py](#real_time_circles_detectionpy)
  - [autonomous_control.py](#autonomous_controlpy)
  - [orientations_and_confirmation.py](#orientations_and_confirmationpy)
- [Installation](#installation)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)


Our approach to achieve autonomous navigation through the circles involves several key steps:

1. Real-time Circle Detection:
   - Capturing frames from the ROV's camera.
   - Detecting circles in the frames using the Hough Circle Transform method.
   - Displaying the detected circles on the frame.

3. Autonomous Control:
Subscribing to the ROV's depth information.
Using a pre-trained circle detection model to identify circle positions.
Calculating the required movement directions (dx, dy, dz) to navigate through the detected circles.
Publishing velocity commands to the ROV based on the detected circles and the calculated movement directions.
Adjusting the ROV's path based on feedback from the circle detector to ensure it passes through each circle correctly.
GUI for Orientation and Confirmation:

Allowing the user to set orientations for the circles using radio buttons.
Displaying the current orientations and allowing the user to confirm the detected circles.
If confirmed, the system proceeds with autonomous control using the selected orientations.


## Project Structure

### `real_time_circles_detection.py`
This script captures frames from a camera feed and detects circles in real-time using the `CirclesDetector` class.

#### Functions

- **capture_frames(queue, camera_ip)**: Captures frames from the specified camera feed and puts them in a queue.
- **detect_and_display(queue)**: Detects circles in the frames and displays the feed with detected circles highlighted.
- **main()**: Initializes the queue and starts the processes for frame capturing and circle detection.

### `autonomous_control.py`
This script controls an ROV autonomously using ROS 2, navigating through detected circles.

#### AutomaticController Class

- **depth_callback(msg)**: Callback function for depth subscription.
- **cmd(dx, dy, dz)**: Sets the velocity command for the ROV.
- **timer_callback()**: Periodic function to process the current frame and update the ROV's velocity.

#### Functions

- **main(args=None)**: Initializes the ROS 2 node and starts the camera feed and control loop.

### `orientations_and_confirmation.py`
This script provides a GUI for setting circle orientations and confirming images. It starts the autonomous control script with the specified orientations upon confirmation.

#### GUIsmol Class

- **change_orientations()**: Updates the current orientations based on user input.
- **confirm_circles()**: Confirms the detected circles and starts the autonomous control script if confirmed.

## Installation

1. **Clone the repository**
   ```bash
   git clone https://github.com/yourusername/rov-navigation.git
   cd rov-navigation
