#!/bin/bash

if [ "$EUID" -ne 0 ]; then
    echo "This script can only be run as root"
    exit
fi

ARCH=$(dpkg --print-architecture)
if [ ! "$ARCH" == *"aarch64"* ]; then
    echo "Updating fan control"
    # comes from https://docs.nvidia.com/jetson/archives/r34.1/DeveloperGuide/text/SD/PlatformPowerAndPerformance/JetsonXavierNxSeriesAndJetsonAgxXavierSeries.html?highlight=fan#fan-profile-control
    systemctl stop nvfancontrol
    sed -i.bak -e 's/FAN_DEFAULT_PROFILE.*/FAN_DEFAULT_PROFILE cool/' /etc/nvfancontrol.conf
    rm /var/lib/nvfancontrol/status
    systemctl start nvfancontrol

    echo "Disabling desktop"
    systemctl set-default multi-user.target

    echo "Configuring CAN interfaces"
    # Remove the mttcan blacklist because NVIDIA
    rm /etc/modprobe.d/denylist-mttcan.conf

    # Load kernel modules at startup
    cat >> /etc/modules-load.d/can.conf <<EOF
# Loads modules required for GPIO can interfaces on the jetson
can
can_raw
mttcan
EOF

    # Load in systemd-networkd scripts
    cat > /etc/systemd/network/20-wired.network <<EOF
# Static IP configuration for ethernet port
# Sets the IP address for this computer
[Match]
Name=eth0

[Network]
Address=192.168.1.22/24
Gateway=192.168.1.1
DNS=8.8.8.8 1.1.1.1
EOF

    cat > /etc/systemd/network/50-internal-can.network <<EOF
# Internal CAN Bus (can0) Configuration for Talos
[Match]
Name=can0

[CAN]
BitRate=1M
RestartSec=1s
EOF

    cat > /etc/systemd/network/51-external-can.network <<EOF
# External CAN Bus (can1) Configuration for Talos
[Match]
Name=can1

[CAN]
BitRate=250K
RestartSec=1s
EOF

    cat > /etc/udev/rules.d/50-can-qlen.rules <<EOF
# Configures CAN bus transmit queue length
SUBSYSTEM=="net", ACTION=="add|change", KERNEL=="can0" ATTR{tx_queue_len}="1000"
SUBSYSTEM=="net", ACTION=="add|change", KERNEL=="can1" ATTR{tx_queue_len}="1000"
EOF

    # Switch from NetworkManager to systemd-networkd on next reboot
    systemctl disable NetworkManager
    systemctl mask NetworkManager
    systemctl unmask systemd-networkd
    systemctl enable systemd-networkd

else
    echo "This script is not running on jetson hardware. Aborting"
    exit
fi