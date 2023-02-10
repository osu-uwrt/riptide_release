#!/bin/bash

# Update ~/.bashrc File

s1="source ~/osu-uwrt/jetson_install/install/setup.bash"
s2="source ~/colcon_deploy/install/setup.bash"

# Add appropriate lines to the bashrc if they do not exist
if ! grep -q "$s1" ~/.bashrc || ! grep -q "$s2" ~/.bashrc; then
    sed -i "/setup.bash/d" ~/.bashrc
    echo $s1 >> ~/.bashrc
    echo $s2 >> ~/.bashrc
fi