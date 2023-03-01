# Release Repository for Riptide Vehicle Software
Below are some important instructions on how to make the best use of this repository and quickly deploy and manage vehicle software.

## Pre Pool Test
1. Before a pool test the versions of the submodules should be updated to the versions intended to be tested in the water. 
2. This repository can then be committed with the new submodule versions to the master branch. 
3. Leave for the pool on time. (no, really, try to leave on time)

## At Pool Test
This repository should be the workspace deployed to the robot via the [colcon deploy extension](https://github.com/osu-uwrt/riptide_setup/tree/humble/colcon_riptide). It supports the normal colcon package selection args to make the process more customizable and lower iteration times. By setting up your terminal inside this repository `~/osu-uwrt/release`, you should be able to deploy all needed packages to the robot as you work through bugs. It is also recommended that you run any GUI tools out of this workspace and not the normal development version, just to make sure the configurations are synchronized correctly.

## Post Pool Test
After a pool test, this repository should be tagged with the date of the test, then released on github. This is to serve as a backup for future pool tests, should software need to be rolled back. Before tagging, any commits against the submodules should be committed, followed by a submodule update commiit to bump the submodule to the new version. The tag convention should be the date of the pool test, and the release should contain any comments about the state of the codebase and or what was tested.

## Robot Software
This repo is set up such that each of the packages found in the `osu-uwrt/development` workspaces should be set as a submodule in this repository. The submodules are placed in the `src` directory and are version pinned to the correct versions running on the robot. This is intended to make controlling the code being tested on the robot a streamlined process. 

## Cross Build Instructions
For cross building ROS distributions to the outdated versions of ubuntu that nvidia has decided to ship on their jetson boards, this repository contains a cross building system. 
See the [Cross Compiling Readme](./scripts/cross_build/README.md) for more information.