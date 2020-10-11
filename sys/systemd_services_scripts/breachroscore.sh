#!/bin/bash

source /opt/ros/melodic/setup.bash
source /home/breach/catkin_ws/devel/setup.bash

export ROS_MASTER_URI=http://192.168.1.15:11311
export ROS_HOSTNAME=192.168.1.15

roslaunch breach core.launch
