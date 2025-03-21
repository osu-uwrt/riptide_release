#/bin/bash

# Update and Upgrade bc why not
sudo apt update && sudo apt -y upgrade

# Make sure universe repo is enabled
sudo apt install -y software-properties-common
sudo add-apt-repository universe

# Make sure curl is installed and install ROS GPG into keyring
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add package repo to apt
sudo echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update Package Repos
sudo apt update

# Install ROS
sudo apt install -y ros-humble-ros-base ros-dev-tools
