#!/bin/bash

# Update ~/.bashrc File

s1="source ~/jetson_install/install/setup.bash"
s2="source ~/colcon_deploy/install/setup.bash"
# Note: Change this to source_discovery_jetson.bash when we want to permenantely enable discovery server
s3="source ~/scripts/discovery_server_scripts/source_discovery_jetson_transitional.bash"

# Add appropriate lines to the bashrc if they do not exist
if ! grep -q "$s1" ~/.bashrc || ! grep -q "$s2" ~/.bashrc; then
    sed -i "/setup.bash/d" ~/.bashrc
    echo $s1 >> ~/.bashrc
    echo $s2 >> ~/.bashrc
    echo $s3 >> ~/.bashrc
fi
