#!/bin/bash

########################################
# This file will add an alias to conditionally enable the discovery server on the jetson.
#
# This file should be sourced in the bashrc when you don't want to have the discovery server always
# enabled. However, when you are ready to transition over to full time discovery server, you are free
# to delete this file and directly source source_discovery_jetson.bash
########################################

function discovery_server_enable () {
    if ! [ -f /tmp/discovery_server_enabled ]; then
        touch /tmp/discovery_server_enabled
    fi

    local script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
    source $script_dir/source_discovery_jetson.bash
    if ! [[ $PS1 =~ "DDS Host: " ]]; then
        # Only set PS1 if we don't have something in there already
        PS1="\[\033[33m\](Discovery Server)\[\033[0m\] $PS1"
    fi
}

# If discovery server enabled set, autorun discovery_server_enable
if [ -f /tmp/discovery_server_enabled ]; then
    discovery_server_enable
    unset -f discovery_server_enable
fi
