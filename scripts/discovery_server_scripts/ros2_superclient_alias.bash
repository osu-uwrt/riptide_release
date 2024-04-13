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


# Sanity check before trying to use this alias
# This is because this inherits the environment variables for the ros2 commands, but explicitly sets the profile xml file
# If these variables diverge (someone tries to clear the ROS_DISCOVERY_SERVER) but doesn't know about this, it'll cause weird results
# Yell at them instead
if [ -z "$ROS_DISCOVERY_SERVER" ]; then
    echo "Discovery server not set!?!"
    echo "Please remove the ros2 alias to prevent your ros2 commands from using the superclient discovery profile"
    echo "Additionally, you'll probably want to restart the ros2 daemon if you had it running with a server previously"
    exit 1
fi

discovery_cfg_host_file="$DISCOVERY_CFG_PATH/host"
discovery_cfg_superclient_xml="$DISCOVERY_CFG_PATH/superclient.xml"

if ! [ -f "$discovery_cfg_host_file" ]; then
    echo "Could not locate discovery server host configuration file"
    echo "Something must have gotten corrupted in the environment!"
    echo "Please purge the environment and retry"
    exit 1
fi

if ! [ -f "$discovery_cfg_superclient_xml" ]; then
    echo "Could not locate discovery server superclient configuration file!"
    echo "Something must have gotten corrupted in the environment!"
    echo "Please purge the environment and retry"
    exit 1
fi

discovery_target="$(cat "$discovery_cfg_host_file")"
discovery_port=11811

# Check that the discovery server is alive, report an error if not
# Don't allow any ros2 commands to run so we don't break ros
if ! "$SCRIPT_DIR/check_discovery_server_alive.bash" "$discovery_target" "$discovery_port"; then
    # Kill the daemon since things will definitely get fragmented
    ros2 daemon stop 2>&1 > /dev/null

    echo "Error: The discovery server '$discovery_target' does not appear to be online!"
    echo "Aborting the requested ros2 command to avoid the network from getting fragmented"
    exit 1
fi

# Run conditionally
if [ "$1" = "launch" ] || [ "$1" = "run" ]; then
    # Don't enable super client if running ros2 run or ros2 launch, since we don't want every node we launch to be a superclient
    ros2 $@
else
    # If it's a different ros command, make it a superclient to give it full introspection access
    FASTRTPS_DEFAULT_PROFILES_FILE="$discovery_cfg_superclient_xml" ros2 $@
fi

