#!/bin/bash

# Update ~/.bashrc File

if [ "$1" = "classic" ];
then
    s1="source /opt/ros/humble/install/setup.bash"
else
    s1="source ~/jetson_install/install/setup.bash"
fi

s2="source ~/colcon_deploy/install/setup.bash"
s3="source ~/scripts/discovery_server_scripts/source_discovery_jetson.bash"

# Add appropriate lines to the bashrc if they do not exist
if ! grep -q "$s1" ~/.bashrc || ! grep -q "$s2" ~/.bashrc || ! grep -q "$s3" ~/.bashrc; then
    sed -i "/setup.bash/d" ~/.bashrc
    echo $s1 >> ~/.bashrc
    echo $s2 >> ~/.bashrc
    echo $s3 >> ~/.bashrc
fi
