#!/bin/bash

if [ "$EUID" -ne 0 ]; then
    echo "This script can only be run as root"
    exit
fi

# install the server

apt update && apt install -y tigervnc-standalone-server

# move the service files
cp ./etc_xstartup /etc/vnc/xstartup
cp ./user_xstartup /home/$SUDO_USER/.vnc/xstartup

# make them executable
chmod +x /etc/vnc/xstartup /home/$SUDO_USER/.vnc/xstartup

# move the
cp "./vncserver@.service" /etc/systemd/system

systemctl daemon-reload
systemctl enable "vncserver@1.service"

echo "VNC setup complete, restart to enable"