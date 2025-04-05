#!/bin/bash

# Super Speedy Fan
echo "Updating fan control"
sudo systemctl stop nvfancontrol
mv /etc/nvfancontrol.conf /etc/nvfancontrol.conf.bkup
cp configs/nvfancontrol.conf /etc/nvfancontrol.conf
sudo rm /var/lib/nvfancontrol/status
sudo systemctl start nvfancontrol

# Configure Network with Systemd Network
echo "Configuring Network"

rm /etc/systemd/network/*.link
rm /etc/systemd/network/*.network

cp configs/systemd/*.link /etc/systemd/network
cp configs/systemd/*.network /etc/systemd/network

# Switch from NetworkManager to systemd-networkd on next reboot
sudo systemctl disable NetworkManager
sudo systemctl mask NetworkManager
sudo systemctl unmask systemd-networkd
sudo systemctl enable systemd-networkd

# add the dvl to the hosts file
sudo sh -c "echo "192.168.1.212    uwrt-dvl" >> /tmp/hosts"

sudo cp configs/systemd/mnt-sd.mount /etc/systemd/system
