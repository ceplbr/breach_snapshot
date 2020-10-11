#!/bin/bash

# CUSTOM COLORS
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

# CHANGE cz PACKAGES TO us
#printf "${RED}Setting package source to US ${NC}\n"
#sudo sed -i 's/cz./us./g' /etc/apt/sources.list

# ADD ROS SOURCE
printf "${RED}Adding ros source ${NC}\n"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

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
printf "${RED}rosdep update ${NC}\n"
rosdep update

# UPDATE .bashrc
printf "${RED}adding lines to .bashr ${NC}\n"
echo " " >> ~/.bashrc
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo " " >> ~/.bashrc
echo "export ROS_MASTER_URI=https://localhost:11311" >> ~/.bashrc
#echo "export ROS_MASTER_URI=https://localhost" >> ~/.bashrc

# SETTING ROS WORKSPACE
printf "${RED}Installing ROS dependencies ${NC}\n"
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential -y

printf "${RED}Setting workspace ${NC}\n"
source ~/.bashrc
source /opt/ros/melodic/setup.bash

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
git clone https://github.com/zatakon/BREACH.git .

cd ~/catkin_ws
source devel/setup.bash

# INSTALL DEPENDENCIES
sudo apt install ros-melodic-move-base ros-melodic-serial ros-melodic-rosserial ros-melodic-gmapping -y

# BUILD ALL
cd ~/catkin_ws
catkin_make


