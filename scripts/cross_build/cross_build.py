import os, stat, hashlib, json, tarfile, getpass, yaml
from time import sleep
import argparse
from subprocess import Popen, PIPE, STDOUT
import urllib.request

USER_HOME = os.path.expanduser("~")
USER_NAME = getpass.getuser()

META_EXTENSIONS = [".pkgs", ".meta", ".repos"]
CLEAN_DIRS = ["build", "install", "log"]

DOCKERFILE_TEMPLATE = """FROM osrf/ubuntu_arm64:{0}
# set dpkg to non-interactive install
ENV DEBIAN_FRONTEND noninteractive

# Upgrade system packages to latest
RUN apt update && apt upgrade -y 

# generate the UTF-8 locales
RUN apt update && apt install locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG en_US.UTF-8

# make sure we have pip and all that too for colcon and rosdep
RUN apt update && apt install curl gnupg lsb-release -y
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
 http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
 | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt update && apt install python3-rosdep2 python3-colcon-common-extensions python3-pip git -y

# Make the build directory
RUN mkdir /ros_build
"""

BUILD_SCRIPT_TEMPLATE = """#!/bin/bash

# first download the apt packages we need
apt install -y $(cat /ros_build/meta.pkgs)
if [[ $? -ne 0 ]]; then
    echo "failed to find one or more packages"
    exit -1
fi

cd /ros_build

# run rosdep to make sure things look okay package wise
rosdep update
rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"

# meta file performs the following patches as of 8/7/22
# for python_orocos_kdl_vendor: missing python3 https://www.reddit.com/r/cmake/comments/otxfb4/comment/h724us5/  feck you from a salty engineer <3
# for rclpy: missing python3 and issues with pybind11 https://github.com/ros2/rclpy/issues/920
# for tf2_py, tf2_geometry_msgs, rosbag2_py missing python3 

{0}

# this build forces a full clean configure and rebuild of all packages. this should make everything relatively compliant assuming
# the current repos remain buildable (failure https://github.com/micro-ROS/micro-ROS-Agent.gits do make it into release sometimes...)
colcon build --merge-install --metas /l4t/toolchain/aarch64-buildroot-linux-gnu/sysroot/home/meta.meta {1}

"""
# --event-handlers console_direct+
# -DCMAKE_VERBOSE_MAKEFILE=ON


# the main part of the recipie
def main():
    # look at argparse to see if we should use a previous config
    parser = argparse.ArgumentParser(description="Build a ROS meta for jetson native use")
    parser.add_argument("-p", "--prev", action="store_true", help="Use previously written config")
    parser.add_argument("--cfg", type=str, help="config file to use when loading", default="/tmp/cross_build_cf.yaml", required=False)
    args = parser.parse_args()

    settings = {}
    if(args.prev):
        print("Loading prior config")
        with open(args.cfg) as cfgFile:
            settings = yaml.safe_load(cfgFile)

        print("Using config settings: ")
        print(settings)

    else:
        # get the configuration for what we are about to do
        settings = configure_menu()

    # give the user a moment to commit
    print("Configuring build settings\n")
    sleep(2.5)

    # build the builder container
    make_docker_image(settings)

    # pull the sources locally
    checkout_sources(settings)

    # invoke the builder container
    build_sources(settings)

    if settings["archive"]:
        make_archive(settings)


# takes the user through a configuration process for running the cross build
def configure_menu() -> dict:
    confDict = {}

    # prompt for target OS build
    osOptions = get_docker_tags("osrf", "ubuntu_arm64")
    osSel = select_menu(osOptions, "Ubuntu distribution")
    confDict["base_os"] = osOptions[osSel]

    # prompt for which meta file set to use from the unique set
    metaDir = os.path.join(USER_HOME, "osu-uwrt", "release", "scripts", "meta_files")
    confDict["meta_origin"] = metaDir
    metaFiles = sorted(list(set([file.split(".")[0] for file in os.listdir(metaDir)])))
    metaSel = select_menu(metaFiles, "ROS2 Build")
    confDict["meta_name"] = metaFiles[metaSel]

    clean = input("Run a clean build? [y/n]: ")
    confDict["clean"] = clean.lower() == "y"
    if(not confDict["clean"]):
        print("Skipping cleaning prior build artifacts")

    archive = input("Create final archive for installation? [y/n]: ")
    confDict["archive"] = archive.lower() == "y"
    if(not confDict["archive"]):
        print("Skipping archive creation. Build will be final step")

    print("Select the highest package to build (default is all)")
    confDict["tgt_pkg"] = input("Highest package name: ")
    if(confDict["tgt_pkg"] != ""):
        print("Selecting target package " + confDict["tgt_pkg"])
    
    # set the cross build workspace dir
    confDict["cross_dir"] = os.path.join(USER_HOME, "osu-uwrt", "jetson_install")

    # write the config to tmp as a yaml file
    with open("/tmp/cross_build_cf.yaml", "w") as file:
        confDump = yaml.dump(confDict)
        file.write(confDump)

    return confDict

def make_docker_image(settings: dict):
    baseOs = settings["base_os"]

    # write the dockerfile to the local directory
    dockerfile = DOCKERFILE_TEMPLATE.format(baseOs, "")
    with open("Dockerfile", "w") as file:
        file.write(dockerfile)
    
    # now build the container and tag it
    containerTag = f"arm64_ros_builder_{baseOs}:latest"
    dockerCmd = ["docker", "build", ".", "--tag", containerTag]

    print("Building cross compile container")
    exitCode, _ = execute(dockerCmd, True)
    if(exitCode != 0):
        print("Error: Docker container build failure. see output logs")
        exit(-4)

    settings["docker_tag"] = containerTag
    return settings

def checkout_sources(settings: dict):
    # make the checkout directory
    print("Making cross build dir")
    crossSrcs = os.path.join(settings["cross_dir"], "src")
    os.makedirs(crossSrcs, exist_ok=True)

    # copy the meta files that we want to build
    print("Copying build meta files")
    for extension in META_EXTENSIONS:
        originFile = os.path.join(settings["meta_origin"], settings["meta_name"] + extension)
        destFile = os.path.join(USER_HOME, "osu-uwrt", "jetson_install", f"meta{extension}")
        meta_copy_overwrite(originFile, destFile)


    print("Preparing checkout sources for build")

    # Read the old hash if it exists
    old_hash = ""
    old_hash_file = "/tmp/jetson_xc_checkout"
    if(os.path.isfile(old_hash_file)):
        with open(old_hash_file, "r") as old_hash_content:
            old_hash = old_hash_content.read()

    new_hash = md5_file(os.path.join(settings["cross_dir"], "meta.repos"))
    # print(f"Old Hash: {old_hash}")
    # print(f"New Hash: {new_hash}")
        
    # check that we have an old hash and that it matches
    if(old_hash != "" and old_hash == new_hash):
        print("Meta source definition files have not changed, skipping checkout")
        
    # either the old hash isnt there, or they dont match
    else:
        print("Meta source files differ, pulling sources")

        # run the vcs clone
        exitCode, _ = execute(["vcs", "import", crossSrcs, "--input", os.path.join(settings["cross_dir"], "meta.repos")], False)
        if(exitCode != 0):
            print("VCS import / update failed. You may not be connected to the internet.")
            print("Proceeding without checkout in 5 seconds")
            sleep(5)

        # write the current hash to the file
        print(f"Writing hash file for next checkout")
        with open(old_hash_file, "w") as hash_file: 
            hash_file.write(str(new_hash))

    # make sure we have the target package before going any further
    if(settings["tgt_pkg"] != ""):
        exitCode, packages = execute(["colcon", "list"], cwd=settings["cross_dir"])

        # check for a zero exit code, which means colcon ran okay
        if(exitCode != 0):
            print("Warning: failed to run colcon list when specified target package.\nBuild may fail with package not found")

        else:
            # clean up the list of packages we got back
            cleanPkgs = []
            for pkg in packages:
                cleanPkgs.append(pkg.split("\t")[0])

            # make sure we find the package in there
            if(not settings["tgt_pkg"] in cleanPkgs):
                print("Error: Could not find package " + settings["tgt_pkg"])
                print("Found: ")
                for pkg in cleanPkgs:
                    print(f"\t{pkg}")
                exit(-5)
    

def build_sources(settings: dict):
    # first we make a bash script to handle the bit inside the container
    print("Generating build script")
    workDir = settings["cross_dir"]

    # Detect if a clean has been requested
    cleanCmd = "# this is used to clean the build directory if requested"
    if(settings["clean"]):
        print("Cleaning build directories")

        cleanCmd += "\nrm -rf "
        for dir in CLEAN_DIRS:
            cleanCmd += dir + " "

    # detect if we have specified a target package
    tgtPkgArg = ""
    if(settings["tgt_pkg"] != ""):
        tgtPkgArg = "--packages-up-to " + settings["tgt_pkg"]

    # Format the build script template
    build_script = BUILD_SCRIPT_TEMPLATE.format(cleanCmd, tgtPkgArg)

    # write the build script file
    scriptFile = os.path.join(workDir, "build.bash")
    with open(scriptFile, "w") as script:
        script.write(build_script)

    # make the script executable
    st = os.stat(scriptFile)
    os.chmod(scriptFile, st.st_mode | stat.S_IEXEC)

    print("Building sources")
    dockerCmd = [
        "docker", "run", "-v", 
        f"{workDir}:/ros_build", 
        settings["docker_tag"], "/bin/bash", 
        "/ros_build/build.bash"
    ]

    flat = ""
    for cmd in dockerCmd:
        flat += cmd + " "

    exitCode, _ = execute(dockerCmd, True)
    if(exitCode != 0):
        print("Docker build failure, see output logs for more information")

def make_archive(settings: dict):
    # grab the working directory
    workDir = settings["cross_dir"]

    print("Owning build archive")
    userPass = getpass.getpass(prompt="Enter your password to re-own the build archive: ")

    # need to own the work directory first
    execute(["chown", "-R", USER_NAME, workDir], sudo=True, userPass=userPass)

    archiveFile = os.path.join(USER_HOME, "osu-uwrt", settings["meta_name"] + "_built.tar.gz")

    # now we can create the archive tarball
    print("Creating archive of build results. This may take a few minutes")
    with tarfile.open(archiveFile, mode="w:gz") as archive:
        archive.add(workDir, arcname="./" + settings["meta_name"] + "_built")
        # archive.list()

    print("Archive complete!")

        
    











##########################################################################################
### Helper functions below this point used for hashing, user input and shell execution ###
##########################################################################################

# a special copy function to copy file content and overwrite for the meta files 
def meta_copy_overwrite(src_file: os.PathLike, dest_file: os.PathLike):
    # we need to make sure the src file exists
    if(not os.path.isfile(src_file)):
        print(f"Error: could not find source file {src_file}")
        exit(-3)

    # read the source file
    src_content = []
    with open(src_file, "r") as src:
        src_content = [line for line in src]

    # write / overwrite the dest file
    with open(dest_file, "w") as dest:
        dest.writelines(src_content)

# chunk wise md5 hash on files
def md5_file(file: os.PathLike) -> int:
    BUF_SIZE = 65536  # lets read stuff in 64kb chunks!

    md5 = hashlib.md5()

    with open(file, 'rb') as f:
        while True:
            data = f.read(BUF_SIZE)
            if not data:
                break

            md5.update(data)

    return md5.hexdigest()

# Function for requesting the availiable tags of a docker repository in Docker Hub
def get_docker_tags(namespace: str, repo: str) -> list:
    request = urllib.request.urlopen(f"https://hub.docker.com/v2/namespaces/{namespace}/repositories/{repo}/tags")
    with request as response:
        data = response.read()

        dataDict = json.loads(data)

        tags = []
        if(len(dataDict["results"]) > 0):
            for tag in dataDict["results"]:
                tags.append(tag["name"])

            tags = sorted(tags)
        else:
            print("Error pulling tags, This may be due to having no internet connection")

    return tags
    
# helper function for building the config menus when doing selections
def select_menu(options: list, list_name: str) -> int:
    print(f"Found availiable {list_name} options:")

    selection = -1
    for i in range(len(options)):
        print(f"{i}.\t{options[i]}")
    if(len(options) > 1):
        selection = int(input(f"Select the {list_name} to use:\n"))
    elif(len(options) > 0):
        selection = 0
        print(f"Selected {options[0]} as it was the only option")
    print("\n")
    return selection

# version of call that pipes stdout back
def execute(fullCmd, printOut=False, sudo=False, userPass="", cwd=None):
    if(sudo): fullCmd = ["sudo", "-S"] + fullCmd
    if printOut: print(fullCmd)
    stdoutText = []

    proc = Popen(fullCmd, stdout=PIPE, stderr=STDOUT, universal_newlines=True, cwd=cwd)

    if(sudo):
        stdoutText, _ = proc.communicate(userPass + "\n")
        if printOut: print(stdoutText)

    else:
        for line in iter(proc.stdout.readline, ""):
            stdoutText.append(line)
            if printOut: print(line)

    proc.stdout.close()
    retCode = proc.wait()
    return(retCode, stdoutText)

if __name__ == "__main__":
    main()