#!/bin/bash

echo "Sourcing fastdds server configuration"

ros2 daemon stop

source /home/ros/uwrt-config/source/client.bash

bash -c "ros2 daemon stop"
bash -c "fastdds shm clean"
bash -c "sudo rm -rf /dev/shm/*fastrtps*"

#export DISCOVERY_SERVER_CMD="/home/ros/fastdds-rmw/install/fastdds/bin/fastdds discovery -i 0 -l 0.0.0.0 -t 0.0.0.0"
