#!/bin/bash

# CUSTOM COLORS
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

printf "Breach install script -Part 3- System config\n"

# Add user to dialout so it can access com ports
printf "${RED} Adding user to dialout group ${NC}\n"
sudo adduser $USER dialout

# Start SSH server
printf "${RED} SSH server ${NC}\n"
sudo apt install openssh-server -y

# Copy udev rules
printf "${RED} Applying udev rules ${NC}\n"
sudo cp ../udev/20-breach-usb.rules /etc/udev/rules.d/20-breach-usb.rules

# Copy systemd services
printf "${RED} Applying systemd services ${NC}\n"
sudo cp ../systemd_services/breachamclteb.service /etc/systemd/system/breachamclteb.service
sudo cp ../systemd_services/breachhandlelogs.service /etc/systemd/system/breachhandlelogs.service
sudo cp ../systemd_services/breachroscore.service /etc/systemd/system/breachroscore.service

cp ../systemd_services_scripts/breachamclservice.sh /home/breach/catkin_ws/src/breach/scripts/breachamclservice.sh
cp ../systemd_services_scripts/breachroscore.sh /home/breach/catkin_ws/src/breach/scripts/breachroscore.sh
cp ../systemd_services_scripts/breachhandlelogs.sh /home/breach/catkin_ws/src/breach/scripts/breachhandlelogs.sh

# Make executable if not already
sudo chmod +x /home/breach/catkin_ws/src/breach/scripts/breachamclservice.sh
sudo chmod +x /home/breach/catkin_ws/src/breach/scripts/breachroscore.sh
sudo chmod +x /home/breach/catkin_ws/src/breach/scripts/breachhandlelogs.sh

# Start services
sudo systemctl start breachhandlelogs.service
sudo systemctl enable breachhandlelogs.service
sudo systemctl status breachhandlelogs.service --no-pager

sudo systemctl start breachroscore.service
sudo systemctl enable breachroscore.service
sudo systemctl status breachroscore.service --no-pager

printf "${RED} Installation -Part 3- done! ${NC}\n"
