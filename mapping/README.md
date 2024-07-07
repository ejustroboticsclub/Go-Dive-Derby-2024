# Underwater 2D Mapping

## Overview
This project involves an underwater 2D mapping task using a Remotely Operated Vehicle (ROV). The ROV's path is tracked in a simulated pool environment, and it includes a detection model trained to identify and place specific shapes (pipes, cubes, and cuboids) on the map when detected.

## Table of Contents
- [Shape Detection Model](#shape-detection-model)
- [Tracking ROV's Trajectory](#tracking-rovs-trajectory)
- [Manual Mapping](#manual-mapping)
- [Autonomous Mapping](#autonomous-mapping)
- [Scripts](#scripts)
- [Requirements](#requirements)

___

## Shape Detection Model
We used YOLOv8 for the shape detection model. Our dataset was collected by designing specific cubes, cuboids, and pipes, which were then placed underwater. Photos of these shapes were taken using our ROV's camera. The data was labeled using Roboflow. 
- The data used in training is located at `data/processed`.
- The best trained model is located at `models/best.pt`.
___

## Tracking ROV's Trajectory
In our project, we visualize real-time navigation of our Remotely Operated Vehicle (ROV) in a simulated underwater pool environment. We've integrated Pygame for graphical rendering and ROS (Robot Operating System) for communication and data handling. Our setup tracks the ROV's movement by subscribing to velocity commands and IMU (Inertial Measurement Unit) data. This allows us to monitor and analyze the ROV's path in a dynamic underwater setting, providing crucial insights for our mapping and exploration task.

___

## Manual Mapping
In our manual approach for the mapping task, we employ a Pygame-based graphical interface to simulate an underwater environment for tomb mapping. This interface allows us to interactively place and visualize key elements such as coffins, treasures, and papyrus rolls within a defined map area. By utilizing 3D shapes and intuitive button controls, users can strategically position these items on the map, facilitating the creation of a detailed visual representation. This manual mapping approach not only enhances our understanding of spatial relationships but also provides a practical framework for planning and executing mapping tasks in underwater scenarios. </br> </br>
The source code is located at `src/manual-mapping`
___

## Autonomous Mapping
Our autonomous approach combines the ROV path tracker with the shape detection model to facilitate underwater mapping tasks. As the ROV navigates through the environment, the path tracker continuously monitors its movement, updating a real-time map. Simultaneously, the shape detection model identifies specific objects such as pipes, cubes, and cuboids. Upon detection, these shapes are automatically placed on the map, providing an accurate and dynamic representation of the underwater terrain. This integrated approach enhances efficiency and accuracy in mapping operations, leveraging both robotic navigation capabilities and advanced object recognition technology. </br> </br>
The source code is located at `src/autonomous-mapping`.
___

## Scripts

1. **`src/autonomous-mapping/shape_detection.py`**
   - This Python script implements a `ShapeDetector` class. The class provides methods to load the model, perform inference on video frames or images, and draw bounding boxes around detected objects with corresponding class labels and confidence scores.

2. **`src/autonomous-mapping/save_predicted_shapes.py`**
   - This Python script utilizes multiprocessing to concurrently capture frames from an IP camera feed of our ROV and perform real-time shape detection using the pre-trained shape detection model.
   - It integrates OpenCV for video capture and processing, and a custom `ShapeDetector` class for detecting and visualizing shapes such as cubes, cuboids, and pipes.
   - Detected shapes are recorded to a text file (`shape.txt`) and displayed on-screen with bounding boxes.
    
3. **`src/autonomous-mapping/rov_path_visualizer.py`**
   - This Python script integrates ROS (Robot Operating System) with Pygame for visualizing the trajectory of a Remotely Operated Vehicle (ROV) in a simulated underwater environment.
   - It subscribes to velocity commands and IMU data from ROS topics, allowing real-time updates of the ROV's position and orientation on the graphical interface.
   - It subscribes to messages indicating the presence of specific shapes (`/Cube`, `/Cuboid`, `/pipe`), which are then visually represented on the map. This visualization aids in monitoring the ROV's path and the distribution of detected shapes during underwater exploration tasks.

4. **`src/autonomous-mapping/shape_publisher.py`**
   - This Python script implements a ROS 2 node (`ShapeDetectorNode`) that subscribes to a topic (`/ROV/shape`) to receive Boolean messages.
   - It reads shape detection results from a file (`shape.txt`) and publishes boolean messages to corresponding topics (`/Cube`, `/Cuboid`, `/pipe`) based on detected shapes.
   - Upon publishing a True message to any of these topics (Cube, Cuboid, pipe), the corresponding shape will be plotted on the map.
    
5. **`src/manual-mapping/basic_manual_mapping_pygame.py`**
   - This Python script uses Pygame to simulate an interactive map for underwater tomb mapping.
   - It allows users to place and visualize various items such as a cuboid (coffin), cube (treasure chest), and circles (papyrus rolls) on a designated map area.
  
6. **`src/manual-mapping/updated_pygame_using_arrows.py`**
   - This script creates an interactive Pygame application for an underwater tomb mapping simulation, allowing the user to place various items on a map and control a Remotely Operated Vehicle (ROV).
   - Users can move in the map using the arrow keys on the keyboard, and when the Enter key is pressed, the selected shape is placed on the map.
  
___

## Requirements

- **`Python 3.7+`**
- **`rospy`**: ROS Python client library for ROS communication.
- **`tf`**: ROS library for transformations between coordinate frames.
- **`rclpy`**: ROS 2 Python client library for ROS 2 communication.
- **`ultralytics`**: (YOLOv8): Deep learning library for object detection.
- **`opencv-python`**: (cv2): OpenCV library for computer vision tasks.
- **`multiprocessing`**: Python module for spawning processes using an API similar to threading.
- **`pygame`**: Library for creating multimedia applications like games and simulations in Python.

