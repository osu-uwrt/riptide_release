#!/bin/bash

export XAUTHORITY=$HOME/.Xauthority
sudo bash subscripts/configure-jetpack.bash

bash subscripts/install-ros.bash
bash subscripts/configure-fastdds.bash

bash subscripts/install-zed-sdk.bash

bash subscripts/rosdep.bash

sudo reboot
