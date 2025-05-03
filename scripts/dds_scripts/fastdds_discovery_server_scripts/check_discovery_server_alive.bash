#!/bin/bash

set -e
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

########################################
# Performs steps to see if the discovery server is alive
# This is a utility script called by other scripts in this folder
#
# This script iss required as it is nontrivial to check if a discovery server
# is alive. See more info in the udp_icmp_check program to learn more, but
# what's needed is two steps, which are performed in this script
#
# This checks with a high degree of certainty if a discovery server is online
########################################


if [ -z "$1" ] || [ -z "$2" ]; then
    echo "Usage: $0 [host] [port]"
    exit 127
fi

discovery_target="$1"
discovery_port="$2"

# Try to ping (sending packet every 200ms, quiet output, trying for 3 seconds)
# This will succeed after the first packet goes through
# If all of the packets fail, then it will report a discovery error
# This is the first step (as the second step assumes that the host will send ICMP errors if there's no server)
# So, this makes sures the host is first alive and will respond to pings, before assuming it'll give icmp errors
ping_results=$(ping -i 0.2 -q -w 3 -c 1 "$discovery_target" 2> /dev/null)
ping_rc=$?
if [[ $ping_rc -ne 0 ]]; then
    exit 1
fi

# Get the max time it took to ping the host, so we can tune the ICMP timeout properly
# This makes it so if we're on wifi and it's super slow, we can wait up to the 200ms or so for a response,
# but if we're over ethernet, the ros2 commands feel snappy
max_ping_f="$(echo "$ping_results" | grep 'rtt min/avg/max/mdev' | cut -d'=' -f2 | cut -d'/' -f3)"
# Round the max ping time to next decimal
max_ping_d="$(printf '%.0f\n' "$max_ping_f")"

# Set minimum value so we'll wait for at least 10 milliseconds for an ICMP error response
if [[ $max_ping_d -lt 5 ]]; then
    max_ping_d=5
fi

# Now, if we haven't compiled the udp_icmp_check program yet, do so
if ! [ -f "$SCRIPT_DIR/udp_icmp_check" ]; then
    gcc -o $SCRIPT_DIR/udp_icmp_check $SCRIPT_DIR/udp_icmp_check.c
fi

# Run the icmp check, waiting up to 2x the ping time in milliseconds for an ICMP port unreachable error
if ! "$SCRIPT_DIR/udp_icmp_check" "$discovery_target" "$discovery_port" "$(($max_ping_d * 2))"; then
    exit 2
fi

exit 0
