########################################
# DON'T SOURCE THIS FILE DIRECTOY
# This contains shared functions to be used on both host and jetson sourcing scripts
# It removes code duplication, at the cost of spreading it across files
########################################

# This function is a wrapper for the _python_argcomplete called by ros2
# This allows tab complete for ros2 to run as a superclient, so tab complete shows all the nodes
function _python_argcomplete_ros2_wrapper {
    local saved_profiles_file="$FASTRTPS_DEFAULT_PROFILES_FILE"

    export FASTRTPS_DEFAULT_PROFILES_FILE="$DISCOVERY_CFG_PATH/superclient.xml"
    if ! [ -f "$FASTRTPS_DEFAULT_PROFILES_FILE" ]; then
        echo "Error doing tab complete, superclient cfg not found" >&2
    else
        _python_argcomplete $@
    fi

    if ! [ -z "$saved_profiles_file" ]; then
        FASTRTPS_DEFAULT_PROFILES_FILE=$saved_profiles_file
    else
        unset FASTRTPS_DEFAULT_PROFILES_FILE
    fi
}

# Utility function to report an error while sourcing the discovery server
# Needs to be unset in programs that source this file
function report_fail {
    echo
    echo -e "\033[1;33m!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\033[0m"
    for var in "$@"; do
        printf "\033[1;33m%s\033[0m\n" "$var"
    done
    echo -e "\033[1;33m!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\033[0m"
    echo

    # Report that the Discovery server is broken
    if ! [[ $PS1 =~ "DDS Host: " ]]; then
        # Only set PS1 if we don't have something in there already
        PS1="\[\033[91m\](DDS Host: <BROKEN>)\[\033[0m\] $PS1"
    fi
}

function check_ddshost_ready() {
    local script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

    # Check to make sure all the required variables are in the environment
    if [ -z "$ROS_DISCOVERY_SERVER" ] || [ -z "$DISCOVERY_CFG_PATH" ]; then
        echo "Discovery server variable not set!"
        echo "Something must have gotten corrupted in the environment!"
        echo "Please purge the environment and retry"
        return 1
    fi

    local discovery_cfg_host_file="$DISCOVERY_CFG_PATH/host"
    local discovery_cfg_superclient_xml="$DISCOVERY_CFG_PATH/superclient.xml"

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

    local discovery_target="$(cat "$discovery_cfg_host_file")"
    local discovery_port=11811

    # Check that the discovery server is alive, report an error if not
    # Don't allow any ros2 commands to run so we don't break ros
    if ! "$script_dir/check_discovery_server_alive.bash" "$discovery_target" "$discovery_port"; then
        # Kill the daemon since things will definitely get fragmented
        ros2 daemon stop 2>&1 > /dev/null

        echo "Error: The discovery server '$discovery_target' does not appear to be online!"
        echo "Aborting the requested ros command to avoid the network from getting fragmented"
        return 1
    fi

    return 0
}

# Function which will run the command as a superclient on the machine
# Useful for debug programs which need to run as a superclient
function run_as_superclient {
    if [ -z "$1" ]; then
        echo "Usage: run_as_superclient [command name] [command args...]"
        return 1
    fi

    # Make sure the ddshost is ready for things
    if ! check_ddshost_ready; then
        return 1
    fi

    # Now run command as a superclient
    FASTRTPS_DEFAULT_PROFILES_FILE="$DISCOVERY_CFG_PATH/superclient.xml" $@
}
