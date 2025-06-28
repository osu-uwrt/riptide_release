#!/bin/bash

set -e

########################################
# DO NOT CALL THIS SCRIPT DIRECTLY
# It should instead be set as an alias for the ros2 command by source_discovery.bash
#
# This script will override most ros2 commands to run as a superclient
# This is needed since, when using a discovery server, the clients (aka nodes) will only receive topics they're subscribed to
# Stuff like ros2 topic info and tab complete will break, since they aren't subscribed to the topics they're trying to list
# See https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html#ros-2-introspection for more info
########################################

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
source "$SCRIPT_DIR/source_shared.bash"


if [ "$1" == "launch" ] || [ "$1" == "run" ]; then
    # Running as launch or run, we need to see if these are special scripts that need superclient
    if [ "$2" == "plotjuggler" ]; then
        # Run plotjuggler as superclient
        run_as_superclient=1;
    elif [ "$2" == "riptide_rviz" ]; then
        # Run riptide_rviz as superclient
        run_as_superclient=1
    else
        # Anything else is standard node, run as normal client
        run_as_superclient=0
    fi
else
    # All other ros commands can run as superclient
    run_as_superclient=1
fi

if [ "$run_as_superclient" == "1" ]; then
    run_as_superclient ros2 $@
else
    # Even if not running as superclient, still run sanity checks to make sure we're connected to ROS
    if ! check_ddshost_ready; then
        exit 1
    fi

    ros2 $@
fi

