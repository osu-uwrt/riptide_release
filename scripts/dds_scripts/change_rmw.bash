#
# RMW CHANGE SCRIPT
# This script prompts the user for the name of an RMW and applies it to the system
#

source /home/$USER/osu-uwrt/release/scripts/dds_scripts/rmw_vars.bash

numdirs=$(ls -1 | wc -l)

echo "There are $numdirs rmw options available:"
echo
ls $DDS_SCRIPTS_DIR -1 --color=never --hide=*.* #this lists options but NOT files
echo

target_rmw=""
while [ -z "$target_rmw" ]; do
    read -p "Please enter the name of the rmw to use. (Default: $DEFAULT_RMW): " target_rmw

    if [ -z "$target_rmw" ]; then
        target_rmw=$DEFAULT_RMW
    fi

    #check that the source_rmw script exists
    srcrmwfile="$DDS_SCRIPTS_DIR/$target_rmw/source_rmw.bash"
    if [ ! -f "$srcrmwfile" ]; then
        echo "Option $target_rmw is missing its source_rmw.bash script (should be located at $srcrmwfile). Please select a different option."
        target_rmw="" #force while loop to go back again
    fi
done

#now have a valid, actionable dds. Ask user to set it as default
echo

echo "Setting $target_rmw as the default RMW."
mkdir -p ~/.uwrt
echo $target_rmw > $DEFAULT_RMW_FILE

source $APPLY_RMW_SCRIPT
