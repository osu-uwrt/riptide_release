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

## Jetson Target Setup
This repo also contains a python script capable of setting up an Nvidia Jetson target computer with a ROS build created by the [cross building scripts](./scripts/cross_build/README.md) in this repo. For this to work properly, the Jetson and this laptop must both have a valid internet connection. It will do all of the nessecary configuration for the jetson, install ros and add local relationships for your user. 
1. Setup the jetson IP into your hosts file.
2. Have a copy of the ROS tar you wish to install in your downloads folder.
3. Install fabric for python via `pip install fabric` for ssh connections
4. Run the `remote_install.py`and follow the configuration prompts. the flashing will begin after the flashing process starts
5. for subsequent re-runs, a `-p` flag can be used to re-run the previous configuration without needing to go through the menu.

## Cross Build Instructions
For cross building ROS distributions to the outdated versions of ubuntu that nvidia has decided to ship on their jetson boards, this repository contains a cross building system. 
See the [Cross Compiling Readme](./scripts/cross_build/README.md) for more information.




## Helpful Commands
Command to start RViz for Talos:
```
ros2 launch riptide_rviz rviz_start.launch.py robot:=talos
```

Restart the launch service on the robot:
```
sudo systemctl restart ros2_launch_service.service
```

View console of the robot over ssh:
```
journalctl -f -u ros2_launch_service.service -o cat
```

Command to set the map transform to the tag:
```
ros2 action send_goal /talos/map/model_tf chameleon_tf_msgs/action/ModelFrame -f "{monitor_parent: 'talos_base_link', monitor_child: 'estimated_origin_frame'}" 
```

check for connected tf trees: 
```
ros2 run tf2_ros tf2_echo world estimated_origin_frame
```