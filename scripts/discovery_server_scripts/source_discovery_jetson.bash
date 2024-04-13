#!/bin/bash

########################################
# This file will set the appropriate environment variables to enable the discovery server
# running locally on the jetson.
#
# This file should be configured on the jetson by setup_bashrc.bash to run on every login.
# (Well, that's the plan at least). Right now we're going to have an intermediate phase where
# we can configure either discovery server or the direct discovery. So there's a second script
# that will be sourced which will optionally enable this file.
########################################

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
source "$SCRIPT_DIR/source_shared.bash"
export DISCOVERY_CFG_PATH="$SCRIPT_DIR/jetson_discovery_cfg"

function initialize_discovery {
    # Filepath configuration
    local discovery_cfg_host_file="$DISCOVERY_CFG_PATH/host"
    local discovery_cfg_superclient_xml="$DISCOVERY_CFG_PATH/superclient.xml"

    # Directly load configuration from host file
    local discovery_target="$(cat "$discovery_cfg_host_file")"
    local discovery_port=11811
    local discovery_addr="${discovery_target}:${discovery_port}"

    # Do some sanity checks to make sure we have all all the other required files
    if ! [ -f "$discovery_cfg_superclient_xml" ]; then
        report_fail \
            "Could not locate superclient xml configuration alongside the host name configuration!" \
            "Something has gone wrong! Please ensure that you have installed this script correctly"
        return 1
    fi

    # Set the environment variable
    if [ -z "$ROS_DISCOVERY_SERVER" ]; then
        export ROS_DISCOVERY_SERVER="${discovery_addr}"
    elif [ "$ROS_DISCOVERY_SERVER" != "$discovery_addr" ]; then
        report_fail \
            "ROS_DISCOVERY_SERVER variable already set ($ROS_DISCOVERY_SERVER) but does not match configured server ($discovery_addr)!" \
            "Something has gone wrong! Please purge your environment and retry"
        return 1
    fi

    # Set all the aliases we need for this (and unset ddshost if we're realiasing)
    alias ros2="$SCRIPT_DIR/ros2_superclient_alias.bash"
    # Fix the ros2 tab complete to run as superclient
    complete -o default -o nospace -F _python_argcomplete_ros2_wrapper ros2

    # Check that the discovery server is alive, report an error if not
    # This is just a warning though, we'll still let everything work on the orin as though the discovery server is present
    if ! "$SCRIPT_DIR/check_discovery_server_alive.bash" "$discovery_target" "$discovery_port"; then
        report_fail \
            "Error: The discovery server '$discovery_target' does not appear to be online!" \
            "The ROS_DISCOVERY_SERVER has still been set, however you probably will not be able to communicate with the server"
        return 1
    fi

    return 0
}

initialize_discovery
unset -f initialize_discovery
unset -f report_fail
unset SCRIPT_DIR
