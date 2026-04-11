# rosfleet

ROS-based Fleet Management System for multi-AGV task allocation, parking assignment, and obstacle-aware coordination in a factory environment.

## Overview

`rosfleet` is a multi-AGV fleet management project developed for factory intralogistics and autonomous material transp([github.com](https://github.com/MhokWorawat/rosfleet/blob/main/fleetmanagement/package.xml))multiple AGVs, assigns missions, selects the most suitable vehicle based on path distance, manages parking spaces, and monitors obstacle detections and emergency states through ROS topics and services.

This project was developed as part of a Mechatronics Engineering senior project focused on **Fleet Management System for AGV**.

## Key Features

* Multi-AGV task assignment for 4 AGVs
* Automatic AGV selection based on planned path length
* Parking space allocation and reservation logic
* Real-time AGV status tracking
* Obstacle monitoring from YOLO-based detections
* Emergency status handling
* ROS-based communication using publishers, subscribers, and services
* Integration with navigation, AMCL, A* planning, odometry, LiDAR, and camera packages

## System Architecture

The repository is organized around a ROS workspace structure with several packages related to localization, navigation, perception, and fleet coordination.

Main modules include:

* **fleetmanagement** – mission handling, AGV selection, parking logic, and fleet coordination
* **navigation** – navigation stack configuration
* **amcl** – localization setup
* **astar** – path planning support
* **odometry** – robot motion estimation
* **yolov5_ros** – object detection integration
* **usb_cam / ydlidar_ros_driver** – perception hardware drivers

## Fleet Management Logic

The core fleet management node is responsible for:

1. Receiving mission requests
2. Checking available AGVs
3. Estimating route length through `/move_base/make_plan`
4. Selecting the closest or most appropriate AGV
5. Publishing assigned tasks
6. Monitoring parking requests and parking slot availability
7. Broadcasting AGV status and object detection summaries

## Tech Stack

* **ROS 1**
* **Python**
* **C++**
* **AMCL**
* **move_base**
* **A* path planning**
* **YOLOv5 ROS integration**
* **LiDAR + camera-based perception**

## Repository Structure

```bash
rosfleet/
├── fleetmanagement/
│   ├── scripts/
│   │   ├── fleetmanagement.py
│   │   ├── AGV01_management.py
│   │   ├── AGV02_management.py
│   │   ├── AGV03_management.py
│   │   ├── AGV04_management.py
│   │   └── gui_fleetmanagement.py
│   ├── data_path/
│   ├── data_paths/
│   ├── graph_paths/
│   ├── image/
│   ├── package.xml
│   └── CMakeLists.txt
├── navigation/
├── amcl/
├── astar/
├── odometry/
├── usb_cam/
├── ydlidar_ros_driver/
├── yolov5_ros/
└── ...
```

## Example ROS Interfaces

### Subscribed Topics

* `/agv01/amcl_pose`
* `/agv02/amcl_pose`
* `/agv03/amcl_pose`
* `/agv04/amcl_pose`
* `/mission`
* `/request_parking`
* `/agv_status`
* `/agv01/yolov5/detections` to `/agv04/yolov5/detections`

### Published Topics

* `/counter`
* `/agv_status`
* `/object_detection`
* `/closest_parking`
* `/task`

### Services

* `/<agv_namespace>/move_base/make_plan`

## Installation

### Prerequisites

* Ubuntu with ROS 1 installed
* Catkin workspace configured
* Required ROS dependencies installed

### Build

```bash
cd ~/catkin_ws/src
git clone https://github.com/MhokWorawat/rosfleet.git
cd ..
catkin_make
source devel/setup.bash
```

## Run

Because this repository contains multiple packages and platform-specific configurations, launch steps may vary depending on your robot setup. A typical flow is:

```bash
roscore
roslaunch <localization_package> <localization.launch>
roslaunch <navigation_package> <navigation.launch>
rosrun fleetmanagement fleetmanagement.py
```

## Demo Suggestions

To make the repository stronger for recruiters and collaborators, add the following assets:

* System architecture diagram
* AGV map / factory layout image
* GIF or short demo video of mission assignment
* Screenshot of fleet management GUI
* Example mission flow: Automatic -> Station A -> Station D

## Project Highlights

* Designed for real factory-style AGV coordination
* Combines navigation, localization, perception, and task scheduling
* Uses path-based decision making instead of only Euclidean distance
* Structured around practical ROS communication between multiple robots

## Recommended Improvements

To make this repository look more professional:

1. Add a project banner image at the top of the README
2. Replace placeholder metadata in `package.xml`
3. Add a `LICENSE` file
4. Add `requirements` / dependency notes for each major package
5. Add screenshots and demo GIFs
6. Remove temporary files such as `__pycache__`
7. Rename unclear files like `ss.py` and `test.py` to descriptive names
8. Add launch instructions and sample commands
9. Add architecture and topic-flow diagrams
10. Pin 1–2 best screenshots in the README

## License

Please choose and add a license file, such as MIT, Apache-2.0, or BSD-3-Clause, depending on how you want others to use your work.

## Author

**Worawat Naowan**

* Mechatronics Engineer
* Focused on Robotics, Automation, and AGV Fleet Management
* GitHub: [MhokWorawat](https://github.com/MhokWorawat)

---

If you are using this repository for portfolio or job applications, keep the README concise, visual, and outcome-focused. Recruiters should understand the project within 20–30 seconds of opening the page.
