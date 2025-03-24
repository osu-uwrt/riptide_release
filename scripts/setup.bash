#!/bin/bash

bash configure-jetpack.bash
bash install-ros.bash
bash install-zenoh.bash
bash install-zed-sdk.bash

sudo reboot
