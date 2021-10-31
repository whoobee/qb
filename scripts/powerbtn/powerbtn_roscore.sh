#!/usr/bin/env bash

source /opt/ros/noetic/setup.bash
source /home/whoobee/qb/ros_ws/devel/setup.bash
export QB_IP_ADDR=$(hostname -I | awk '{print $1}')
export ROS_MASTER_URI=http://${QB_IP_ADDR}:11311
export ROS_HOSTNAME=${QB_IP_ADDR}
export ROS_IP=${QB_IP_ADDR}

roscore