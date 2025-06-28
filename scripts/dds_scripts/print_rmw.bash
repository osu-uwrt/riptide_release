#
# PRINT RMW SCRIPT
# This script prints the currently active RMW
#

source /home/$USER/osu-uwrt/release/scripts/dds_scripts/rmw_vars.bash
active_rmw=$(cat $DEFAULT_RMW_FILE)
echo "Using RMW $active_rmw"
echo
echo "RMW source script located at $DDS_SCRIPTS_DIR/$active_rmw/source_rmw.bash"
echo "RMW_IMPLEMENTATION environment variable is set to $RMW_IMPLEMENTATION"
