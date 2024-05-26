#!/bin/bash

# make sure the workspace is sourced
source /opt/ros/humble/install/setup.bash && source /root/ros2_ws/install/local_setup.bash 

# Wait for mavros to setup connection
sleep 5

# run zed camera
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm publish_tf:=false publish_map_tf:=false ros_params_override_path:=/root/ros2_ws/src/april_tag_pkg/config/zedm.yaml &

# wait for zed camera to fully start
sleep 5

# start the april_tag_sender node
ros2 run april_tag_pkg april_tag_sender &

# keep the container running
wait $!