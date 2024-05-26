
ARG IMAGE_NAME=docker.io/xtrana/ros2:humble-base-mavros
FROM ${IMAGE_NAME}

# we need setuptools==58.2.0 to build python packages.
RUN pip install setuptools==58.2.0

# copy the package
COPY drone_follower_pkg /root/ros2_ws/src

# build the package
WORKDIR /root/ros2_ws/
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build

# source the workspace
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \ 
    && echo "source /root/ros2_ws/install/local_setup.bash" >> ~/.bashrc

CMD ["bash"]