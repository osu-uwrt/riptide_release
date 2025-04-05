#!/bin/bash

sudo apt update

sudo rosdep init
rosdep update

rosdep install -y -r -i --from-paths $HOME/dependencies
rosdep install -y -r -i --from-paths $HOME/colcon_deploy
