#!/bin/bash


echo "Updating fan control"
# comes from https://docs.nvidia.com/jetson/archives/r34.1/DeveloperGuide/text/SD/PlatformPowerAndPerformance/JetsonXavierNxSeriesAndJetsonAgxXavierSeries.html?highlight=fan#fan-profile-control
sudo systemctl stop nvfancontrol
sudo sed -i.bak -e 's/FAN_DEFAULT_PROFILE.*/FAN_DEFAULT_PROFILE cool/' /etc/nvfancontrol.conf
sudo rm /var/lib/nvfancontrol/status
sudo systemctl start nvfancontrol

echo "Disabling desktop"
sudo systemctl set-default multi-user.target

echo "Configuring CAN interfaces"
# Remove the mttcan blacklist because NVIDIA
rm /etc/modprobe.d/denylist-mttcan.conf


# Load kernel modules at startup
cat >> /etc/modules-load.d/can.conf << EOF
# Loads modules required for GPIO can interfaces on the jetson
can
can_raw
mttcan
EOF

cat >> /etc/systemd/network/20-wired-net.network  << EOF
# Static IP configuration for ethernet port
# Sets the IP address for this computer
[Match]
Name=eth0

[Network]
Address=${DEVICE_IP}/24
Gateway=192.168.1.1
DNS=8.8.8.8 1.1.1.1
EOF

sudo cp configs/51-external-can.network /etc/systemd/network
sudo cp configs/50-can-qlen.rules /etc/udev/rules.d

# Switch from NetworkManager to systemd-networkd on next reboot
sudo systemctl disable NetworkManager
sudo systemctl mask NetworkManager
sudo systemctl unmask systemd-networkd
sudo systemctl enable systemd-networkd

# add to dialout
sudo usermod -aG dialout $USER

# add the dvl to the hosts file
sudo sh -c "echo "192.168.1.212    uwrt-dvl" >> /tmp/hosts"

sudo cp config/mnt-sd.mount /etc/systemd/system
sudo cp configs/99-automount-sd.rules /etc/udev/rules.d


