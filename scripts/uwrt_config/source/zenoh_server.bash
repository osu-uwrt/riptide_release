#!/bin/bash

echo "Sourcing zenoh server configuration"
source /home/ros/uwrt-config/source/client.bash
export DISCOVERY_SERVER_CMD="ros2 run rmw_zenoh_cpp rmw_zenohd"
