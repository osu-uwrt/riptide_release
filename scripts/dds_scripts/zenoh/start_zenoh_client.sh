#!/usr/bin/env bash
# ~/bin/start_zenoh_client.sh

# 1) Choose between Release vs Development install
if [ -f /tmp/source_release ]; then
    echo "Sourcing Release"
    source "$HOME/osu-uwrt/release/install/setup.bash"
else
    echo "Sourcing Development"
    source "$HOME/osu-uwrt/development/dependencies/install/setup.bash"
    source "$HOME/osu-uwrt/development/software/install/setup.bash"
fi

# 2) Common environment
export XAUTHORITY="$HOME/.Xauthority"
export RMW_ZENOH_CONFIG_URI="$HOME/osu-uwrt/release/scripts/dds_scripts/zenoh/zenoh_router_test.json5"

# 3) Source the ROS2 RMW-Zenoh workspace overlay
source "$HOME/rmw_zenoh/install/setup.bash"
# 4) Source any additional ROS config
source "$HOME/osu-uwrt/release/scripts/dds_scripts/config_rmw.bash"

# 5) Finally, replace this script with the router
exec ros2 run rmw_zenoh_cpp rmw_zenohd --config /home/uwrt/osu-uwrt/release/scripts/dds_scripts/zenoh/zenoh_router_test.json5
