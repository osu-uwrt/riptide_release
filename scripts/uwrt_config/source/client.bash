#!/bin/bash

# Source ROS, Deps, and Riptide
source /opt/ros/humble/setup.bash
source /home/ros/rmw_zenoh_cpp/install/setup.bash
source /home/ros/zed-sdk/zed_wrapper/install/setup.bash
source /home/ros/osu-uwrt/release/scripts/dds_scripts/config_rmw.bash
source /home/ros/colcon_deploy/install/setup.bash
source /home/ros/dependencies/micro-ros-agent/install/setup.bash

# Set up RMW
#if [ "$RMW_IMPLEMENTATION" = "rmw_fastrtps_cpp" ];
#then
#    source /home/ros/uwrt-config/source/client_fastdds.bash
#fi
