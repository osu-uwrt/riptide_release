#!/bin/bash

bash configure-jetpack.bash
bash install-ros.bash
bash install-zenoh.bash

sudo reboot
