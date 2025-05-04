#
# UWRT ROS MIDDLE-WARE (RMW) CONFIG SCRIPT
# This script must be sourced at the start of every terminal for rmw to work properly
#

# System default rmw is stored in ~/.uwrt/default_rmw. Check if exists and set rmw to that
# Otherwise ask user what default rmw should be

source /home/$USER/osu-uwrt/release/scripts/dds_scripts/rmw_vars.bash

target_rmw=""

if [ -f $DEFAULT_RMW_FILE ]; then
    source $APPLY_RMW_SCRIPT
else
    numdirs=$(ls -1 | wc -l)
    if [ $numdirs -eq 1 ]; then
        echo "Setting RMW to $target_rmw because it is the only option"
        echo $(ls) > $DEFAULT_RMW_FILE
    else
        echo
        echo "System default RMW not set!"
        echo
        
        source $CHANGE_RMW_SCRIPT

    fi
fi


#set aliases to commands
alias silencermwconf='source $SILENCE_RMW_SCRIPT'
alias changermw='source $CHANGE_RMW_SCRIPT'
alias rmw='source $PRINT_RMW_SCRIPT'
