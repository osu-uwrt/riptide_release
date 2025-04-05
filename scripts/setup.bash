#!/bin/bash

bash subscripts/configure-jetpack.bash
bash subscripts/install-ros.bash
bash subscripts/configure-fastdds.bash
bash subscripts/install-zed-sdk.bash

sudo reboot
