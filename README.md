

---

# drone_vision_landing

Visual Landing and Target Following Using AprilTags

## Overview

This project focuses on visual landing and target following using AprilTags. The detection is performed using the [Isaac ROS AprilTag](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag) package on an NVIDIA Jetson platform.

## Apriltag Detection Setup

Follow these steps to set up AprilTag detection:

### 1. Set Up Isaac ROS Environment

First, set up the Isaac ROS environment by following the instructions [here](https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html).

Clone the necessary repositories into your workspace:

```bash
cd ${ISAAC_ROS_WS}/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag.git
```

### 2. Customize and Run the Docker Image

Customize the Docker image according to your setup by following the guide [here](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_common/index.html). Then, run the Docker container:

```bash
cd ${ISAAC_ROS_WS}/src/isaac_ros_common
./scripts/run_dev.sh
```

### 3. Install Dependencies

Install the required ROS 2 package for AprilTag detection:

```bash
sudo apt-get install -y ros-humble-isaac-ros-apriltag
```

You might need to change the DDS to cyclone for compability with other docker containers.
```bash
sudo apt update
sudo apt install ros-humble-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
```

### 4. Run the Launch File

Run the launch file to start the AprilTag detection. Adjust the launch parameters as needed for your setup. Example launch files can be found [here](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag/tree/main/isaac_ros_apriltag/launch). By default, the tag size is set to 0.22 and the tag family is 36h11.

```bash
ros2 launch isaac_ros_apriltag isaac_ros_apriltag.launch.py
```

To view detections, open a second terminal inside the Docker container and run the following:

```bash
cd ${ISAAC_ROS_WS}/src/isaac_ros_common
./scripts/run_dev.sh
```

---
