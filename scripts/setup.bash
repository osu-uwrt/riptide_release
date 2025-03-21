#!/bin/bash

bash configure-jetpack.bash
bash install-ros.bash
bash install-zenoh.bash

sudo bash vnc_server/install_server.bash

sudo reboot
