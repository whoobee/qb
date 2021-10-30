#!/usr/bin/env bash

source /opt/ros/melodic/setup.bash
source /home/whoobee/qb/ros_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.0.180:11311
export ROS_HOSTNAME=192.168.0.180
export ROS_IP=192.168.0.180

roscore