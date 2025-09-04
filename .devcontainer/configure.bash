#!/bin/bash

cd /wokspaces/riptide_release

if [[ ! -e ~/.configured ]]; then
    # Add this file to bashc
    echo "source /workspaces/riptide_release/.devcontainer/setup.bash" >> ~/.bashrc

    # Init & Pull Submodules
    git submodule init
    git pull --recurse-submodules

    # Update Packages
    apt update && apt upgrade -y

    # Apiltag ROS breaks rosdep
    rm -r /workspaces/riptide_release/src/apriltag_ros

    # Install Deps
    rosdep install -iry --from-paths /workspaces/riptide_release/src

    touch ~/.configured
fi

exit 0
