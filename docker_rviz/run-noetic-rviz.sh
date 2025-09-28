#! /usr/bin/env bash
#
# Run container and launch rviz directly
#
# Presumes that you have orangepi IP in your /etc/hosts or otherwise have DNS resolution working.

docker run -it --rm \
    -e DISPLAY=$DISPLAY \
    -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
    -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
    -e DISALBE_ROS1_EOL_WARNINGS=1 \
    -e ROS_MASTER_URI=http://orangepi:11311 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /mnt/wslg:/mnt/wslg \
    ros-noetic-rviz \
    exec-rviz-defaults.sh
