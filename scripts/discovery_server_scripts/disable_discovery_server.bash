#!/bin/bash

########################################
# Removes the discovery server from the local environment
# This script is configured as an alias by source_discovery.bash
#
# To use, just call 'deactivate_ddshost' in any terminal that has the dds host configured.
#
# It's not as important that this does full sanity checks, since we
# don't waste pool time if someone forgets a step REMOVING a discovery server
# (since they won't be on the robot anymore). They just waste their own debugging time.
#
# So, this just clears the required variables, and asks the user to close everything out
# It's up to them if they get everything properly.
########################################

function disable_discovery {
    if [ -z "$DISCOVERY_CFG_PATH" ]; then
        echo "Error: Discovery server config path not set."
        echo "Please ensure that you have sourced source_discovery.bash before running this script"
        return 1
    fi

    if ! [ -d "$DISCOVERY_CFG_PATH" ]; then
        echo "Discovery server directory does not exist, if you still have a prompt, restart your shell"
        return
    fi

    # Clear the discovery configuration directory
    rm -rf "$DISCOVERY_CFG_PATH"

    # Stop the ros2 daemon
    # Bypass the alias so we don't get the checks yelling at us (since we just purged the config directory)
    \ros2 daemon stop 2>&1 > /dev/null

    # Clean any zombie shared memory transports
    fastdds shm clean 2>&1 > /dev/null

    echo "Successfully disabled the discovery server!"
    echo
    echo -e "\033[1;33mRestart all of your terminals to ensure that you are running a fresh environment!\033[0m"

    # Loop to force them to close out this terminal
    while true; do
        read
    done
}

disable_discovery
unset -f disable_discovery
