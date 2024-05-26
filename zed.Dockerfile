FROM xtrana/ros2:l4t-35.4-zed-desktop-4.1

# install deps
RUN pip install setuptools==58.2.0

# Copy the package
COPY april_tag_pkg /root/ros2_ws/src/april_tag_pkg

WORKDIR /root/ros2_ws

# build the package
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/install/setup.bash && \
    colcon build --packages-select april_tag_pkg"

RUN echo "source /opt/ros/humble/install/setup.bash" >> ~/.bashrc \ 
    && echo "source /root/ros2_ws/install/local_setup.bash" >> ~/.bashrc

CMD ["bash"]