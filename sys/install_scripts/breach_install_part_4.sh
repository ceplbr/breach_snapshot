#!/bin/bash

# CUSTOM COLORS
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

printf "Breach install script -Part 4- Robot config\n"

printf "\nSorry, this is the manual part :( \n\n"

printf "1. Change IP settings in ~/.bashrc file\n"
printf "Change line to 'export ROS_MASTER_URI=http://192.168.1.X:11311'\n"
printf "Add line 'export ROS_HOSTNAME=192.168.1.X'\n"
printf "2. Change robot config in ~/catkin_ws/src/breach/config/robot_config.yaml file\n"
printf "3. Reboot PC with 'sudo reboot'\n"

printf "Installation -Part 4- done!\n"
