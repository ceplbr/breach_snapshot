#!/bin/bash
printf "Step 1\n"
UBUNTU_VER=$(lsb_release -sc)
ROS_VER=melodic
[ "$UBUNTU_VER" = "bionic" ] || exit 1

# CUSTOM COLORS
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

# CHANGE cz PACKAGES TO us
#printf "${RED}Setting package source to US ${NC}\n"
#sudo sed -i 's/cz./us./g' /etc/apt/sources.list
printf "Step 2\n"
# ADD ROS SOURCE
printf "${RED}Adding ros source ${NC}\n"
echo "deb http://packages.ros.org/ros/ubuntu $UBUNTU_VER main" > /tmp/$$-deb
sudo mv /tmp/$$-deb /etc/apt/sources.list.d/ros-latest.list
sudo apt install -y curl
curl -k https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
#sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

printf "Step 3\n"
# INSTALL AND UPDATE
printf "${RED}apt update ${NC}\n"
sudo apt update
printf "${RED}Installing ROS ${NC}\n"
sudo apt install ros-${ROS_VER}-desktop-full -y
printf "${RED}Upgrade all ${NC}\n"
sudo apt upgrade -y
printf "Step 4\n"
# ROSDEP INIT AND UPGRADE
printf "${RED}rosdep init ${NC}\n"
sudo rosdep init
printf "${RED}rosdep upgrade ${NC}\n"
rosdep update
printf "Step 5\n"
# UPDATE .bashrc
printf "${RED}adding lines to .bashr ${NC}\n"
echo " " >> ~/.bashrc
echo "source /opt/ros/${ROS_VER}/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo " " >> ~/.bashrc
echo "export ROS_MASTER_URI=https://localhost:11311" >> ~/.bashrc
echo "export ROS_IP=`hostname -I | awk '{print $1}'`" >> ~/.bashrc
#echo "export ROS_MASTER_URI=https://localhost" >> ~/.bashrc
printf "Step 6\n"
# SETTING ROS WORKSPACE
printf "${RED}Installing ROS dependencies ${NC}\n"
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential rviz -y
sudo apt install git
printf "Step 7\n"
printf "${RED}Setting workspace ${NC}\n"
source ~/.bashrc
source /opt/ros/${ROS_VER}/setup.bash
printf "Step 8\n"
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
printf "Step 9\n"
catkin_make
printf "Step 10\n"
# END OF INSTALATION
printf "\n ${RED}now gow to catkin_ws workspace using${NC}\n"
printf "${RED}  >> cd catkin_ws${NC}\n"
printf "${RED}and build (multiple times) workspace using ${NC}\n"
printf "${RED}  >> catkin_make${NC}\n"
printf "${RED}until succes${NC}\n"

# GIT CLONE
cd ~/catkin_ws/src
git clone https://github.com/zatakon/BREACH.git .
printf "Step 11\n"
cd ~/catkin_ws
source devel/setup.bash
printf "Step 12\n"
# INSTALL DEPENDENCIES
sudo apt install ros-${ROS_VER}-move-base ros-${ROS_VER}-serial ros-${ROS_VER}-gmapping -y
printf "Step 13\n"
# BUILD ALL
cd ~/catkin_ws
catkin_make