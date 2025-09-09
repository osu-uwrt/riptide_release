#!/bin/bash

echo Configuring Workspace

cd /wokspaces/riptide_release

if [[ ! -e ~/.configured ]]; then



    # Add this file to bashc
    echo "source /workspaces/riptide_release/.devcontainer/setup.bash" >> ~/.bashrc


    # Init & Pull Submodules
    git submodule init
    git pull --recurse-submodules

    # Update Packages
    sudo apt update && sudo apt upgrade -y

    # Install Clang
    apt install -y lsb-release wget software-properties-common gnupg
    bash -c "$(wget -O - https://apt.llvm.org/llvm.sh)"

    # Apiltag ROS breaks rosdep
    rm -r /workspaces/riptide_release/src/apriltag_ros

    # Install Deps
    sudo rosdep install -iry --from-paths /workspaces/riptide_release/src

    touch ~/.configured
fi
