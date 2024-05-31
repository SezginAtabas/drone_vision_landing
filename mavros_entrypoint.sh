#!/bin/bash

# make sure the workspace is sourced
source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/local_setup.bash

sleep 1

# launch mavros node
ros2 launch drone_follower_pkg mavros.launch.py &

# wait for zed camera and mavros.
sleep 50

# drone setup and takeoff
ros2 run drone_follower_pkg drone_follower_node

wait $!