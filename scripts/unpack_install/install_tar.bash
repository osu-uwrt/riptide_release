#!/bin/bash 

if [ $# -ge 2 ]; then
    export ROS_DISTRO=$1
    ROS_TAR=$2
else
    echo "This script needs two arguments, ROS_DISTRO, and ROS_TAR"
    echo "./install_tar.bash <ROS_DISTRO> <ROS_TAR>"
    exit
fi

# set to ros2
export ROS_VERSION=2

echo "Unpacking tar archive $ROS_TAR for install"

cd ~

# extract archive
tar -xf $ROS_TAR

# after extraction run apt against the meta file
echo "Installing meta deps"
apt install -y $(cat ~/osu_uwrt/jetson_install/meta.pkgs)
if [[ $? -ne 0 ]]; then
    echo "failed to find one or more packages"
    exit -1
fi

echo "Invoking rosdep"

# run rosdep against the packages to try to resolve the rest
cd ~/osu_uwrt/jetson_install
rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"