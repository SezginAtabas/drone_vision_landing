
# drone_vision_landing

**Visual Landing and Target Following Using AprilTags**

## Overview

`drone_vision_landing` is a project designed for visual landing and target tracking applications using drones, specifically employing AprilTags for precise localization. It's been thoroughly tested on a QuadCopter equipped with CubeOrange running ArduPilot 4.5 firmware.

### Hardware Requirements:
- **NVIDIA Jetson** (as a companion computer)
- **Stereolabs ZED Camera** - Compatible models include [ZED, ZED Mini, ZED 2](https://www.stereolabs.com/).

### Software Dependencies:
- [MAVROS](https://github.com/mavlink/mavros) for communication between ROS and the ArduPilot flight stack.
- [Isaac ROS AprilTag](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag) for AprilTag detection.
- [ZED ROS2 Wrapper](https://github.com/stereolabs/zed-ros2-wrapper) to integrate the ZED cameras with ROS2.
- [Docker](https://www.docker.com/) for containerization.
- [ROS2 Humble](https://docs.ros.org/en/humble/index.html) as the ROS2 distribution.

## Installation Guide

### 1. Set Up Docker
Install Docker on your system by following these [instructions](https://docs.docker.com/get-docker/).

### 2. Configure Isaac for AprilTag Detection
Detailed setup instructions are available [here](https://github.com/SezginAtabas/drone_vision_landing/blob/master/isaac_setup.md).

### 3. Clone the Repository
```bash
git clone https://github.com/SezginAtabas/drone_vision_landing.git
```

### 4. Configure the Parameters
Adjust the MAVROS and camera parameters to fit your hardware setup:
- Modify the `fcu_url` in MAVROS as detailed [here](https://github.com/mavlink/mavros/blob/master/mavros/README.md#installation).
- Update the `camera_model` parameter in the [entrypoint script](https://github.com/SezginAtabas/drone_vision_landing/blob/master/entrypoint.sh) to ensure the ZED node launches with the correct camera model.

### 5. Start the Project Using Docker Compose
Launch the application containers:
```bash
docker compose up
```

### 6. Create and Print an AprilTag
If you haven't already, generate and print an AprilTag from [this generator](https://chev.me/arucogen/).

## Disclaimer

This project is provided for educational and research purposes only. Users are responsible for ensuring they comply with local laws and regulations, particularly those concerning the operation of drones in public or private airspace. The authors of this project are not liable for any damage or legal issues that arise from the use of this software. Always test in controlled environments and ensure that you have appropriate safety measures in place. Use at your own risk.

---

