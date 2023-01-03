# riptide_release

## Cross Build instructions for ROS
This scetion outlines the steps needed to cross-build ROS2 and other packages for an nvidia jetson target.

1. Navigate to https://developer.nvidia.com/embedded/jetson-linux, and download the `Sample Root Filesystem` and the `Bootlin Toolchain gcc x.y`. Leave them in the user's `~/Downloads` directory. 

2. Run the cross build python script as follows: `python3 cross_build.py` in the `scripts/cross_build` directory.

3. Answer the configuration prompts for the application.


### The internal recipe the cross compilier uses
1. Make a directory in osu-uwrt called nvidia, move the root file system and the toolchain inside