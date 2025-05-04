#
# APPLY RMW SCRIPT
# This script actually configures the environment to use the rmw selected by the user.
# The name of the RMW is stored in $target_rmw
#

#print intentions. This part can be silenced because itll happen every time the script runs

source /home/$USER/osu-uwrt/release/scripts/dds_scripts/rmw_vars.bash

target_rmw=$(cat $DEFAULT_RMW_FILE)

if [ ! -f "$SILENCE_FILE" ]; then
    echo
    echo "-------------------------------------------------------------"
    echo "CONFIGURING RMW AS $target_rmw"
    echo
    echo "Available DDS-related commands:"
    echo "silencermwconf : Suppress this print when terminal starts"
    echo "changermw : Change the selected RMW implementation"
    echo "rmw : Print the currently selected RMW"
    echo "-------------------------------------------------------------"
    echo
fi

source $DDS_SCRIPTS_DIR/$target_rmw/source_rmw.bash

ros2 > /tmp/config_rmw_ros_output
if [ $? != 0 ]; then
    echo
    echo
    echo "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"
    echo "@ ERROR SOURCING RMW!!                                        @"
    echo "@ RMW WAS NOT SOURCED CORRECTLY AND IS NOT RECOGNIZED BY ROS! @"
    echo "@ PLEASE RESET YOUR RMW USING THE COMMAND change_rmw!         @"
    echo "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"
    echo
fi
