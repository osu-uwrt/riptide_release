#!/bin/bash

sudo apt update && sudo apt upgrade -y

# Setup the ROS repos
sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update

sudo apt install -y build-essential cmake git python3-colcon-common-extensions python3-colcon-mixin\
python3-flake8 python3-flake8-docstrings python3-pip python3-pytest python3-pytest-cov python3-rosdep \
python3-setuptools python3-vcstool wget bison flex ccache clang lld llvm libc++-dev libc++abi-dev libacl1-dev \
libpython3-dev python3-dev python-is-python3 libboost-python-dev pv

# update mixins
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default

sudo rosdep init && rosdep update
