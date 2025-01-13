#!/usr/bin/bash

#
# This script installs the discovery server and enables it in systemd.
# 
# IMPORTANT: This script is written specifically for Jetson targets using
# the UWRT colcon_riptide plugin (meaning that the remote_launch
# package will be present in the colcon_deploy workspace).
#

echo "Installing and enabling systemd unit"
mkdir -p ~/.config/systemd/user
cp ~/colcon_deploy/src/remote_launch/ros_launcher.service ~/.config/systemd/user #install unit
echo "Changing ownership of user systemd directory to $USER"
chown $USER ~/.config/systemd/user
systemctl daemon-reload #this discovers unit
systemctl enable ros_launcher.service
