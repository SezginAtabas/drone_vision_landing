#!/bin/bash

# make sure the workspace is sourced
source /opt/ros/humble/install/setup.bash && source /root/ros2_ws/install/local_setup.bash

# launch mavros node
ros2 launch drone_control_pkg mavros.launch.py &

wait $!