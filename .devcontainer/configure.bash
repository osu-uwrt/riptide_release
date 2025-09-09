#!/bin/bash

echo Configuring Workspace
wd=${PWD}

if [[ ! -e ~/.configured ]]; then

    # Add this file to bashc
    echo "source ${wd}/.devcontainer/setup.bash" >> ~/.bashrc


    # Init & Pull Submodules
    git submodule update --init
    git pull --recurse-submodules

    # Update Packages
    sudo apt update && sudo apt upgrade -y

    # Install Clang
    apt install -y clang clangd

    # Install Deps
    rosdep install -iry --rosdistro humble --from-paths ${wd}/src

    touch ~/.configured
fi
