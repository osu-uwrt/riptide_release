source /home/$USER/osu-uwrt/release/scripts/dds_scripts/rmw_vars.bash

read -p "Would you like to suppress RMW config terminal output? Enter Y to suppress and N to unsuppress. [Y/n]: " wantsuppress

if [ "$wantsuppress" == "Y" ] || [ "$wantsuppress" == "y" ];
then
    mkdir -p /home/$USER/.uwrt
    touch $SILENCE_FILE
else
    rm -f $SILENCE_FILE
fi
