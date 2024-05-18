#!/bin/bash

# make sure the workspace is sourced
source /opt/ros/humble/install/setup.bash && source /root/ros2_ws/install/local_setup.bash 

# run zed camera
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm &

# wait for zed camera to fully start
sleep 5

# start the april_tag_sender node
ros2 run april_tag_pkg april_tag_sender &

# keep the container running
wait $!


