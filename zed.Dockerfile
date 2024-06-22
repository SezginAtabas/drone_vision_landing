FROM xtrana/ros2:l4t-36.2-zed-desktop-4.1

# install deps
RUN pip install setuptools==58.2.0

# Copy the package
COPY april_tag_pkg /root/ros2_ws/src/april_tag_pkg
COPY isaac_ros_apriltag_interfaces /root/ros2_ws/src/isaac_ros_apriltag_interfaces

WORKDIR /root/ros2_ws

# build the package
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/install/setup.bash && \
    colcon build"

RUN echo "source /opt/ros/humble/install/setup.bash" >> ~/.bashrc \ 
    && echo "source /root/ros2_ws/install/local_setup.bash" >> ~/.bashrc

CMD ["bash"]