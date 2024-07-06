# Underwater 2D Mapping

## Overview
This project involves an underwater 2D mapping task using a Remotely Operated Vehicle (ROV). The ROV's path is tracked in a simulated pool environment, and it includes a detection model trained to identify and place specific shapes (pipes, cubes, and cuboids) on the map when detected.

## Table of Contents
- [Shape Detection Model](#shape-detection-model)
- [Tracking ROV's Trajectory](#tracking-rovs-trajectory)
- [Manual Mapping](#manual-mapping)
- [Autonomous Mapping](#autonomous-mapping)

## Shape Detection Model
We used YOLOv8 for the shape detection model. Our dataset was collected by designing specific cubes, cuboids, and pipes, which were then placed underwater. Photos of these shapes were taken using our ROV's camera. The data was labeled using Roboflow. 
- The data used in training is located at `data/processed`.
- The best trained model is located at `models/best.pt`.

## Tracking ROV's Trajectory
In our project, we visualize real-time navigation of our Remotely Operated Vehicle (ROV) in a simulated underwater pool environment. We've integrated Pygame for graphical rendering and ROS (Robot Operating System) for communication and data handling. Our setup tracks the ROV's movement by subscribing to velocity commands and IMU (Inertial Measurement Unit) data. This allows us to monitor and analyze the ROV's path in a dynamic underwater setting, providing crucial insights for our mapping and exploration task.


## Manual Mapping
In our manual approach for the mapping task, we employ a Pygame-based graphical interface to simulate an underwater environment for tomb mapping. This interface allows us to interactively place and visualize key elements such as coffins, treasures, and papyrus rolls within a defined map area. By utilizing 3D shapes and intuitive button controls, users can strategically position these items on the map, facilitating the creation of a detailed visual representation. This manual mapping approach not only enhances our understanding of spatial relationships but also provides a practical framework for planning and executing mapping tasks in underwater scenarios. </br> </br>
The source code is located at `src/manual-mapping`

## Autonomous Mapping
Our autonomous approach combines the ROV path tracker with the shape detection model to facilitate underwater mapping tasks. As the ROV navigates through the environment, the path tracker continuously monitors its movement, updating a real-time map. Simultaneously, the shape detection model identifies specific objects such as pipes, cubes, and cuboids. Upon detection, these shapes are automatically placed on the map, providing an accurate and dynamic representation of the underwater terrain. This integrated approach enhances efficiency and accuracy in mapping operations, leveraging both robotic navigation capabilities and advanced object recognition technology. </br> </br>
The source code is located at `src/autonomous-mapping`.

