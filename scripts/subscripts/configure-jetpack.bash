#!/bin/bash

# Super Speedy Fan
echo "Updating fan control"
systemctl stop nvfancontrol
mv /etc/nvfancontrol.conf /etc/nvfancontrol.conf.bkup
cp configs/nvfancontrol.conf /etc/nvfancontrol.conf
rm /var/lib/nvfancontrol/status
systemctl start nvfancontrol

# Configure Network with Systemd Network
echo "Configuring Network"

rm /etc/systemd/network/*.link
rm /etc/systemd/network/*.network

cp configs/systemd/*.link /etc/systemd/network
cp configs/systemd/*.network /etc/systemd/network

# Switch from NetworkManager to systemd-networkd on next reboot
systemctl disable NetworkManager
systemctl mask NetworkManager
systemctl unmask systemd-networkd
systemctl enable systemd-networkd

# add the dvl to the hosts file
echo "192.168.1.212    uwrt-dvl" >> /tmp/hosts

cp configs/systemd/mnt-sd.mount /etc/systemd/system

# Install JTOP
sudo pip3 install jetson-stats
sudo pip3 install -U jetson-stats
systemctl enable --now jtop.service

# Set Power Mode to MAX
sed -i.bkup -e "s/PM_CONFIG DEFAULT=2/PM_CONFIG DEFAULT=0/" /etc/nvpmodel.conf
