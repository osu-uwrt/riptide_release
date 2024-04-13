#!/bin/bash

########################################
# Configures the host to use a fastdds discovery server.
# This script is configured as an alias by source_discovery.bash
#
# To use this script, type 'ddshost hostname' where hostname is the host to use as the dds host for your macine.
#
# This performs all the required checks before the switch:
#   - Discovery server is not already configured on this host
#   - The requested discovery server is reachable
#   - There aren't any other running ros nodes on this host
#
# It then places the required files in the discovery config path
# (set to /tmp/discovery_server_cfg) such that any bash shell sourced in
# the future will use this discovery server. It finally performs a check to
# warn if any other bash shells are running on the system (to remind you to source them as well)
########################################

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

function enable_discovery_server {
    if [ -z "$1" ]; then
        echo "Expected hostname argument"
        return 1
    fi

    local discovery_target="$1"
    local discovery_port=11811
    if [ -z "$DISCOVERY_CFG_PATH" ]; then
        echo "Error: Discovery config path not set!"
        echo "Please ensure that source_discovery.bash has been ran"
        return 1
    fi

    # Make sure we don't have any discovery server configured before running this command
    if [ -d "$DISCOVERY_CFG_PATH" ] || (! [ -z "$ROS_DISCOVERY_SERVER" ]); then
        echo "Error: A discovery server has already been configured for this machine"
        echo "Aborting discovery server enable"
        return 1
    fi

    # Make sure we can actually contact the discovery server
    if ! "$SCRIPT_DIR/check_discovery_server_alive.bash" "$discovery_target" "$discovery_port"; then
        echo "Error: The discovery server '$discovery_target' does not appear to be online!"
        echo "Aborting discovery server enable"
        return 1
    fi

    # Kill the ros2 daemon if it running
    if ! command -v ros2 &> /dev/null; then
        echo "Error: The ros2 command is not in the system path, cannot stop the daemon"
        echo "Aborting discovery server enable"
        return 1
    fi
    ros2 daemon stop 2>&1 > /dev/null

    # Clean up any zombie shm ports
    if ! command -v fastdds &> /dev/null; then
        echo "Error: The fastdds command is not in the system path, cannot clean fastdds shared memory objects"
        echo "Aborting discovery server enable"
        return 1
    fi
    fastdds shm clean 2>&1 > /dev/null

    # Make ros is COMPLETELY DEAD on the machine before continuing
    # Check for fastdds shared memory objects
    # However, if we're running on localhost, this check won't work, since the discovery server makes these
    if [ "$discovery_target" == "localhost" ]; then
        echo "NOTE: Skipping check for running shared memory objects, because the fastdds discovery server will make these."
        echo "It's up to you to make sure ROS is dead!"

    elif [[ "$(find /dev/shm -name '*fastrtps*' | wc -l)" -gt 0 ]]; then
        echo "Error: Found fastrtps shared memory objects in /dev/shm"
        echo "A ROS node is still running on this system"
        echo "Please kill all ROS nodes before continuing"
        return 1
    fi

    # Before we do the replace in the superclient template, escape any weird characters that'll break sed
    local escaped_host_replace=$(printf '%s\n' "$discovery_target" | sed -e 's/[\/&]/\\&/g')

    # Create all of the special files for the sourcing scripts to realize we've got a discovery server
    mkdir "$DISCOVERY_CFG_PATH"
    echo "$discovery_target" > "$DISCOVERY_CFG_PATH/host"
    sed "s/\${DISCOVERY_SERVER_ADDR}/$escaped_host_replace/" "$SCRIPT_DIR/superclient_template.xml" > "$DISCOVERY_CFG_PATH/superclient.xml"

    echo "Successfully configured system to use discovery server on host '$discovery_target'"

    # If we have any bash processes running by our user on the system, raise a warning to tell the user to restart it
    # Set ps to match all processes
    local ps_output="$(ps xo pid=,exe=)"
    # Need to do shell count separately, as this spawns a bunch of subshells which messes with the math
    local num_shells="$(echo "$ps_output" | grep "$(realpath "$SHELL")" | wc -l)"
    if [[ "$num_shells" -gt "$SHLVL" ]]; then
        local num_other_shells="$(($num_shells - $SHLVL))"
        echo
        echo -e "\033[1;33mWarning: Detected $num_other_shells other shells running on the system!"
        echo "$ps_output" | grep "$(realpath "$SHELL")" | grep -v "$$"
        echo -e "\033[1;33mClose all other terminal windows before continuing! (Or resource the bashrc in each one)\033[0m"
        echo
    fi
    return 0
}

# Enable the discovery server, then resource the environment
enable_discovery_server $1
source "$SCRIPT_DIR/source_discovery.bash"
unset SCRIPT_DIR
unset -f enable_discovery_server
