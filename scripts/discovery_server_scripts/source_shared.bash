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
