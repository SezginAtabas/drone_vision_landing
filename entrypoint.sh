#!/bin/bash

# make sure the workspace is sourced
source /opt/ros/humble/install/setup.bash && source /root/ros2_ws/install/local_setup.bash 

# Wait for mavros to setup connection
sleep 30

# static publisher for position of the zed camera. 
ros2 run tf2_ros static_transform_publisher --x 0.0 --y 0.98 --z 0.079 --roll 0.022 --pitch 0.073 --yaw 0.165 --frame-id base_link --child-frame-id zed_camera_link &

sleep 1

# run zed camera
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm publish_tf:=false publish_map_tf:=false ros_params_override_path:=/root/ros2_ws/src/april_tag_pkg/config/zedm.yaml &

sleep 1

# start the april_tag_sender node
ros2 run april_tag_pkg april_tag_sender &

sleep 10

ros2 run april_tag_pkg pose_estimator &


# keep the container running
wait $!