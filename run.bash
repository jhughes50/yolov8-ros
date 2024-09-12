#!/bin/bash

# Make sure processes in the container can connect to the x server
#XAUTH=/tmp/.docker.xauth
#if [ ! -f $XAUTH ]
#then
#    touch $XAUTH
#fi
#xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
#if [ -n "$xauth_list" ]
#then
#  echo "$xauth_list" | xauth -f $XAUTH nmerge -
#fi
#chmod a+r $XAUTH
#xhost +
docker run --rm -it --gpus all \
    --privileged    \
    -v "/dev:/dev"  \
    -u $UID \
    -e "TERM=xterm-256color" \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "/home/jason/dtc-dev/yolov8-ros:/home/`whoami`/ws/src/yolov8-ros" \
    -v "/home/jason/ROS/bags:/home/`whoami`/data" \
    --name dtc-jackal-phobos-masking \
    dtc-jackal-phobos:masking \
    bash
