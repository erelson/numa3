#! /usr/bin/env bash
#
# Launch the container, but don't auto-run rviz.
# Exec into it like: docker exec -it ros-noetic-rviz bash

docker run -it --rm \
    -e DISPLAY=$DISPLAY \
    -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
    -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
    -e DISALBE_ROS1_EOL_WARNINGS=1 \
    -e ROS_MASTER_URI=http://orangepi:11311 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ./exec-rviz-defaults.sh:/root/exec-rviz-defaults.sh \
    -v ./numa.rviz:/root/numa.rviz \
    -v /mnt/wslg:/mnt/wslg \
    ros-noetic-rviz
