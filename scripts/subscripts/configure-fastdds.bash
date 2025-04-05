#!/bin/bash

# Directory to Keep Riptide Bashrc stuff together
mkdir $HOME/.riptide_config
touch $HOME/.riptide_config/source-ros

# Create Source File for FastDDS Config
touch $HOME/.riptide_config/source-fastdds

cat > $HOME/.riptide_config/source-fastdds << EOF
RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Set Discovery Server for none daemon tasks
ROS_DISCOVERY_SERVER=$DISCOVERY_SERVER_ADDRESS


toggle_superclient() {

  if [ -z "FASTRTPS_DEFAULT_PROFILES_FILE"]; then
  # Disable Super Client
    unset FASTRTPS_DEFAULT_PROFILES_FILE
  else
    FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.riptide_config/fastdds-config.xml
  fi

  # Restart Daemon to Apply Config Changes
  ros2 daemon stop
  ros2 daemon start
}

alias ddshost toggle_superclient()

EOF

# Add the FastDDS source to the Main Sourcefile
echo "source $HOME/.riptide_config/source-ros"
