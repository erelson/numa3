# Start from the base ROS Noetic image
FROM ros:noetic

# Install RViz and some useful dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-noetic-rviz \
        python3-pyqt5 \
        iputils-ping \
        vim \
        tree \
        && \
    rm -rf /var/lib/apt/lists/*

# Default to bash shell
CMD ["bash"]
