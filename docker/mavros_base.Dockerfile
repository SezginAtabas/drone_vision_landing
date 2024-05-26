ARG ROS_VERSION=humble
ARG ROS_IMAGE=-ros-base-jammy
FROM ros:$ROS_VERSION$ROS_IMAGE

# Install mavros.
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    && rm -rf /var/lib/apt/lists/*

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    ros2 run mavros install_geographiclib_datasets.sh

CMD ["bash"]