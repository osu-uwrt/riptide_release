#!/bin/bash

# Zenoh RMW Repo
# https://github.com/ros2/rmw_zenoh

# Zeno is going to start automatically in the future but
# doesn't currently so we create a systemd unit

# Install Zenoh RMW
sudo apt install -y ros-humble-rmw-zenoh-cpp

mkdir -p ~/zenoh-config
cp configs/zenoh* ~/zenoh-config

cd ~/zenoh-config

# ENV variables for RMW
sudo sh -c "echo RMW_IMPLEMENTATION=rmw_zenoh_cpp >> /etc/environment"
sudo sh -c "echo ZENOH_ROUTER_CONFIG_URI=$PWD/zenoh-router.json5 >> /etc/environment"
sudo sh -c "echo ZENOH_SESSION_CONFIG_URI=$PWD/zenoh-session.json5 >> /etc/environment"

# Use Zenoh for discovery server
sudo sh -c "echo DISCOVERY_SERVER_CMD=ros2 run rmw_zenoh_cpp rmw_zenohd >> /etc/environment"