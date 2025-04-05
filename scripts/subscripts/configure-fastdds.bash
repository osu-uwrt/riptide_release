#!/bin/bash

# Directory to Keep Riptide Bashrc stuff together
mkdir $HOME/.riptide-config
touch $HOME/.riptide-config/source-ros

# Create Source File for FastDDS Config
touch $HOME/.riptide-config/source-fastdds

cat > $HOME/.riptide-config/source-fastdds << EOF
RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Set Discovery Server for none daemon tasks
export ROS_DISCOVERY_SERVER="127.0.0.1:11811"


ddshost() {

  if [ -z "FASTRTPS_DEFAULT_PROFILES_FILE"]; then
  # Disable Super Client
    echo "Disabling Super Client Mode"
    unset FASTRTPS_DEFAULT_PROFILES_FILE
  else
    echo "Enabling Super Client Mode"
    export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.riptide-config/fastdds-config.xml"
  fi

  # Restart Daemon to Apply Config Changes
  ros2 daemon stop
  ros2 daemon start
}
EOF

# Add the FastDDS source to the Main Sourcefile
echo "source $HOME/.riptide-config/source-fastdds" >> $HOME/.riptide-config/source-ros
