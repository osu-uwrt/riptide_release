# the goal of this script is to create, install and configure service units for auto starting our code
from argparse import ArgumentParser
import shutil
from glob import glob
from subprocess import Popen, PIPE, CalledProcessError, call
import os.path

# template ROS Service
SVC_PATTERN = '''
[Unit]
Description=Service to bring up a ros network 
After=network.target

[Service]
Type=simple
ExecStart=/usr/bin/bash /etc/systemd/system/{0}
StandardOutput=inherit
StandardError=inherit
Restart={1}
User={2}


[Install]
WantedBy=multi-user.target
'''

SVC_SCRIPT = '''
#!/bin/bash

source /home/{0}/jetson_install/install/setup.bash
source /home/{0}/colcon_deploy/install/setup.bash

ros2 run {1} {2}
'''

def execute(fullCmd, printOut=False):
    if printOut: print(fullCmd)
    proc = Popen(fullCmd, stdout=PIPE, stderr=PIPE, universal_newlines=True)
    if printOut:
        for line in iter(proc.stdout.readline, ""):
            print(line)
        for errLine in iter(proc.stderr.readline, ""):
            print(f"ERROR: {errLine}")
    proc.stdout.close()
    retCode = proc.wait()
    if retCode:
        raise CalledProcessError(retCode, fullCmd)

if __name__ == "__main__":
    parser = ArgumentParser(description="Setup remote Jetson target")
    # always needed args
    parser.add_argument("name", type=str, help="name of the service to create")
    parser.add_argument("ros_launch_package", type=str, help="name of the package with the launch file inside")
    parser.add_argument("ros_launch_file", type=str, help="name of the launch file to start")
    parser.add_argument("restart_policy", type=str, help="restart policy to use")


    # optional args
    parser.add_argument("-u", "--username", type=str, help="Username to login with", default="ros", required=False)
    args = parser.parse_args()

    SVC_NAME = args.name
    SVC_ROS_PKG = args.ros_launch_package
    SVC_LAUNCH = args.ros_launch_file
    SVC_RESTART = args.restart_policy
    SVC_USER = args.username

    print(f"Creating ros2 launch service {SVC_NAME}")
    print(f"Service will launch {SVC_LAUNCH} from {SVC_ROS_PKG}")
    print(f"Restart policy is {SVC_RESTART}")

    serviceScript = SVC_SCRIPT
    serviceScript = serviceScript.format(SVC_USER, SVC_ROS_PKG, SVC_LAUNCH)
    serviceScriptName = f"ros2_{SVC_NAME}_run.bash"

    serviceFile = SVC_PATTERN
    serviceFile = serviceFile.format(serviceScriptName, SVC_RESTART, SVC_USER)
    serviceFileName = f"ros2_{SVC_NAME}.service"

    tmpSvc = os.path.join("/tmp", serviceFileName)
    print(f"Writing to service file {tmpSvc}")
    with open(tmpSvc, 'w') as file:
        file.write(serviceFile)

    tmpScrpt = os.path.join("/tmp", serviceScriptName)
    print(f"Writing to service file {tmpScrpt}")
    with open(tmpScrpt, 'w') as file:
        file.write(serviceScript)

    try:
        # move the source file into the correct location
        systemFile = os.path.join("/etc", "systemd", "system", serviceFileName)
        systemScript = os.path.join("/etc", "systemd", "system", serviceScriptName)

        print(f"Moving service file to {systemFile}")
        print(f"Moving service script to {systemScript}")

        execute(["sudo", "mv", tmpSvc, systemFile], True)
        execute(["sudo", "mv", tmpScrpt, systemScript], True)

        # register, enable and start the service
        execute(['sudo', 'systemctl', 'daemon-reload'], True)
        execute(['sudo', 'systemctl', 'enable', serviceFileName], True)
        # execute(['sudo', 'systemctl', 'start', serviceFileName], True)

    except CalledProcessError as e:
        print("Error occured while configuring SystemD")
        print(e)




