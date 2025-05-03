# Discovery Server Docs


## Installation on Local Machine

To enable the discovery server on your machine, you just need to add the following line to your bashrc:

    source ~/osu-uwrt/release/scripts/discovery_server_scripts/source_discovery.bash

This will enable all of the aliases and features described.

## Usage on Local Machine

### Connecting to the Discovery Server

To switch your machine to running ROS via a discovery server, you just need to run the `ddshost` alias. This will configure the machine to use whatever hostname is provided as the discovery server.

This has the following requirements:

 * No other ROS nodes are running on your machine
 * The hostname provided has a valid discovery server running
 * It is recommended that you close out all other terminals (so you don't forget to resource)

For example, to connect to the host `orin`, you should run:

    ddshost orin

After this runs, you should see `(DDS Host: orin)` prefixing your bash prompt. **Be sure that all of your terminals have this prefix before running any ros commands.**

### Running as a Superclient

One of the important concepts when using a discovery server is the difference between a *client* and a *superclient*. A client is what a standard node runs as, and is only given information about the topics that it is subscribed to/publishing from. This is useful as it reduces bandwidth (so we don't send all 200 topics and 250 services to every node that comes up).

However, some programs need to know all of the nodes on the network to do debug features (such as ros2 commands, rqt, rviz, and plotjuggler). An environment variable can be set which runs these nodes as superclients, however the challenge is knowing which nodes need to be turned into superclients.

If you want to learn more, check out [the ROS docs](https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html#ros-2-introspection) on the discovery server.

This script handles most of this for you, making ros2 commands (except run/launch), rqt, riptide_rviz launches, and `ros2 run plotjuggler` run as superclients. So, for most cases, you do not need to worry about superclients.

**However,** if you are using a new debug tool that isn't the list above that needs to know all of the nodes on the network, you will need to run it  a superclient. The `run_as_superclient` alias can be prefixed to any command. This will run that command as a superclient, telling the discovery server to report all topics to the node, rather than just topics it's subscribing/publishing to.

For example, let's assume that you have a new launch file that runs plotjuggler in a specific configuration. You need it to be able to see all the topics so plotjuggler works correctly, assuming the launch file is `special_plotjuggler_view.launch.py` in the package `riptide_plotjuggler`, you would run the command as follows:

    run_as_superclient ros2 launch riptide_plotjuggler special_plotjuggler_view.launch.py

### Deactivating the Discovery Server

If you want to disconnect your computer from the discovery server, the easiest way to do so is reboot your computer. However, if you do not want to do that, there's an alias to disable the discovery server.

First, close out all running ros nodes and terminals on your machine. In the last terminal open, run the alias:

    deactivate_ddshost

and all of the environment variables will be cleaned up. After relaunching the terminal, you the discovery server should be removed (and the `DDS Host` prefix will have disappeared from your prompt).


## Installation on a Jetson

This step should already be handled by the setup scripts, but there are two steps to configure the discovery server on a jetson:

1. `source_discovery_jetson.bash` in this directory in the bashrc (or the `_transitional` if you don't want it to be enabled by default)
2. Ensure that the fastdds discovery server is started as part of a systemd service (if the plans for the new launch service go through, it should come up with that, if not it'll be its own discrete service)

## Usage on a Jetson

If you are sourcing the standard `source_discovery_jetson.bash` script, you do not need to do anything to have it run in discovery server mode.

However, if you are sourcing the `source_discovery_jetson_transitional.bash` script, you will need to run the `discovery_server_enable` on the first boot of the orin. Note that there are minimal checks performed, so ensure that no other nodes are running on the machine before this point. To deactivate, you must reboot the machine.
