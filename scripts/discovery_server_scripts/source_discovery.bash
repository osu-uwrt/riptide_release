#!/bin/bash

########################################
# This file will set the appropriate environment variables to enable the discovery server if the
# ddhost alias has been called.
#
# This file should be sourced in your bashrc to ensure every shell inherits the fastdds discovery server
# configuration. This will probably need to be done manually, but just source this and it should work (tm)
########################################

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
source "$SCRIPT_DIR/source_shared.bash"
export DISCOVERY_CFG_PATH="/tmp/discovery_server_cfg"

function initialize_discovery {
    # Filepath configuration
    local discovery_cfg_host_file="$DISCOVERY_CFG_PATH/host"
    local discovery_cfg_superclient_xml="$DISCOVERY_CFG_PATH/superclient.xml"

    if ! [ -f "$discovery_cfg_host_file" ]; then
        # No discovery host file exists, we aren't running as discovery server
        # Configure the environment appropriately
        alias ddshost="source $SCRIPT_DIR/enable_discovery_server.bash"
        if ! [ -z $ROS_DISCOVERY_SERVER ]; then
            unset ROS_DISCOVERY_SERVER
            unalias ros2 >/dev/null 2>/dev/null
            unalias deactivate_ddshost >/dev/null 2>/dev/null
            complete -o default -o nospace -F _python_argcomplete ros2
        fi
        return 0
    fi

    # We found a file, read its contents to get the fastdds discovery host
    local discovery_target="$(cat "$discovery_cfg_host_file")"
    local discovery_port=11811
    local discovery_addr="${discovery_target}:${discovery_port}"

    # Do some sanity checks to make sure we have all all the other required files
    if ! [ -f "$discovery_cfg_superclient_xml" ]; then
        report_fail \
            "Could not locate superclient xml configuration alongside the host name configuration!" \
            "Something has gone wrong! Please purge your environment and retry"
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
    alias deactivate_ddshost="source $SCRIPT_DIR/disable_discovery_server.bash"
    unalias ddshost >/dev/null 2>/dev/null
    # Fix the ros2 tab complete to run as superclient
    complete -o default -o nospace -F _python_argcomplete_ros2_wrapper ros2

    # Check that the discovery server is alive, report an error if not
    if ! "$SCRIPT_DIR/check_discovery_server_alive.bash" "$discovery_target" "$discovery_port"; then
        report_fail \
            "Error: The discovery server '$discovery_target' does not appear to be online!" \
            "The ROS_DISCOVERY_SERVER has still been set, however you probably will not be able to communicate with the server"
        return 1
    fi

    # All checks passed, set the PS1 to report that things are working
    if ! [[ $PS1 =~ "DDS Host: " ]]; then
        # Only set PS1 if we don't have something in there already
        PS1="\[\033[33m\](DDS Host: ${discovery_target})\[\033[0m\] $PS1"
    fi

    return 0
}

initialize_discovery
unset -f initialize_discovery
unset -f report_fail
unset SCRIPT_DIR
