#!/bin/bash

# CUSTOM COLORS
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

# CHANGE cz PACKAGES TO us
printf "${RED}Setting package source to US ${NC}\n"
sudo sed -i 's/cz./us./g' /etc/apt/sources.list

# ADD ROS SOURCE
printf "${RED}Adding ros source ${NC}\n"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

# INSTALL AND UPDATE
printf "${RED}apt update ${NC}\n"
sudo apt update
printf "${RED}Installing ROS ${NC}\n"
sudo apt install ros-melodic-desktop-full -y
printf "${RED}Upgrade all ${NC}\n"
sudo apt upgrade -y

# ROSDEP INIT AND UPGRADE
printf "${RED}rosdep init ${NC}\n"
sudo rosdep init
printf "${RED}rosdep upgrade ${NC}\n"
rosdep update

# UPDATE .bashrc
printf "${RED}adding lines to .bashr ${NC}\n"
echo " " >> ~/.bashrc
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo " " >> ~/.bashrc
echo "export ROS_MASTER_URI=https://localhost:11311" >> ~/.bashrc
echo "export ROS_MASTER_URI=https://localhost" >> ~/.bashrc

# SETTING ROS WORKSPACE
printf "${RED}Installing ROS dependencies ${NC}\n"
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential -y

printf "${RED}Setting workspace ${NC}\n"
source ~/.bashrc
source /opt/ros/kinetic/setup.bash

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make

# END OF INSTALATION
printf "\n ${RED}now gow to catkin_ws workspace using${NC}\n"
printf "${RED}  >> cd catkin_ws${NC}\n"
printf "${RED}and build (multiple times) workspace using ${NC}\n"
printf "${RED}  >> catkin_make${NC}\n"
printf "${RED}until succes${NC}\n"

# GIT CLONE
cd ~/catkin_ws/src
git clone https://github.com/zatakon/BREACH.git

cd ~/catkin_ws
source devel/setup.bash

# INSTALL DEPENDENCIES
sudo apt install ros-kinetic-move-base ros-kinetic-serial ros-kinetic-gmapping -y

# BUILD ALL
cd ~/catkin_ws
catkin_make


