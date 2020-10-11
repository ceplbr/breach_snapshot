#!/bin/bash

# CUSTOM COLORS
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

printf "Breach install script -Part 2- Breach\n"

# Copy files
printf "${RED} Copying files into catkin_ws ${NC}\n"
cp -r ../../sw/catkin_ws/* ~/catkin_ws/

# INSTALL DEPENDENCIES
printf "${RED} Installing Breach dependencies ${NC}\n"
sudo apt install ros-melodic-move-base ros-melodic-serial ros-melodic-rosserial ros-melodic-gmapping -y
sudo apt install ros-melodic-laser-proc ros-melodic-urg-c -y
sudo apt install ros-melodic-map-server ros-melodic-amcl ros-melodic-teb-local-planner -y

# Magic to make it work
printf "${RED} Some magic ${NC}\n"
sudo chmod +x ~/catkin_ws/src/urg_node/cfg/URG.cfg
mkdir -p ~/.ros/bag/breach

# BUILD ALL
printf "${RED} Building catkin_ws${NC}\n"
. /home/breach/.bashrc
. /opt/ros/melodic/setup.sh
cd ~/catkin_ws/
catkin_make -DCATKIN_BLACKLIST_PACKAGES="realsense2_camera"

printf "${RED} Installation -Part 2- done! ${NC}\n"
