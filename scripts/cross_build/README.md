# ROS2 cross builder for jetson
### some quick notes
This build process is intended to be run on ARM64 silicon and not x86. An x86 machine however can be patched to run this by running the docker_arm64_qemu.bash script contained in this directory. 


**Caveat: When running on x86, the process wil take an 8x performance hit**. This means that a build will take about 8x longer to run than an arm counterpart. Even a low spec ARM64 mac as of writing this will outstrip a 16 core desktop by hours.

## Workflow
To run a cross build, use the following steps.
1. Install docker and add your user to the `docker` group to enable sudoless access to docker.
2. Make sure python3 is installed and up to date. All scripts use stdlib functionality so no addtional packages are needed.
3. Run the builder script by invoking `python3 cross_build.py`. This will walk the user through a configuration menu to set up the build. This may take hours to complete the build. 
4. For subsequent re-runs the `-p` flag can be used. This will used the saved settings from the last run. These settings are saved to `/tmp/cross_build_cf.yaml`. The settings file can be copied to a permanent directory and passed in via the argument `--cfg <path/to/cfg/file>` to support pre-configured re-runs.