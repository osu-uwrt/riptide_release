#!/bin/bash

if [ "$EUID" -ne 0 ]; then
    echo "This script can only be run as root"
    exit

# install the server

apt update && sudo apt install -y tigervnc-standalone-server

# move the service files
cp ./etc_xstartup /etc/vnc/xstartup
cp ./user_xstartup ~/.vnc/xstartup

# make them executable
chmod +x /etc/vnc/xstartup ~/.vnc/xstartup

# move the
cp ./vncserver@.service /etc/systemd/system

systemctl daemon-reload
systemctl enable vncserver@1.service