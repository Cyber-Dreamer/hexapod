#!/bin/bash

XSOCK=/tmp/.X11-unix

docker run -it --rm \
 --runtime=nvidia \
 -e DISPLAY=$DISPLAY \
 -v $XSOCK:$XSOCK \
 -v $HOME/.Xauthority:/root/.Xauthority \
 --privileged \
 --net=host \
 osrf/ros:jazzy-desktop-nvidia "$@"