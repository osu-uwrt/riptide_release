#!/bin/python3

import os, yaml, getpass, glob, argparse, time
from posixpath import expanduser
from fabric import Connection, Config
from subprocess import Popen, PIPE, CalledProcessError, call

HOME_DIR = expanduser("~")
BASE_DIR = os.path.join(HOME_DIR, "osu-uwrt", "release", "scripts")
ROS_DISTRO = "humble"

def main():
    # look at argparse to see if we should use a previous config
    parser = argparse.ArgumentParser(description="Build a ROS meta for jetson native use")
    parser.add_argument("-p", "--prev", action="store_true", help="Use previously written config")
    parser.add_argument("--cfg", type=str, help="config file to use when loading", default="/tmp/remote_setup_cf.yaml", required=False)
    args = parser.parse_args()

    # load the settings
    settings = {}
    if(args.prev):
        print("Loading prior config")
        with open(args.cfg) as cfgFile:
            settings = yaml.safe_load(cfgFile)

        print("Using config settings: ")
        print(settings)

    else:
        # get the configuration for what we are about to do
        settings = config_menu()
        print("Writing configuration details")

    time.sleep(2.5)

    # connect to the remote and check for SSH
    connect_ssh(settings)

    print("\n\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    print("!! Preparing to invoke setup scripts on the target !!")
    print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    time.sleep(1.0)

    target_configuration(settings)


# sub section functions
def config_menu():
    confDict = {}

    # scan the hosts file and ask if connecting to these
    targets = scan_hosts()
    print("If target does not appear in this list, add it to the /etc/hosts file")
    targetSel = select_menu(targets, "Remote Target")
    confDict["target_name"] = targets[targetSel]

    # scan for ROS installations availiable
    tarballs = glob.glob(os.path.join(HOME_DIR, "Downloads", f"{ROS_DISTRO}*.tar.gz"))
    if(len(tarballs) > 0):
        print("If the desired ROS tarball is not found place it in this user's downloads directory\n(It must be a GZipped tar archive)")
        tarSel = select_menu(tarballs, "ROS Tarball")
        confDict["tarball_name"] = tarballs[tarSel]
    else:
        print("No ROS tars found in users Downloads directory. Skipping tar install")
        confDict["tarball_name"] = None

    # query for remote login info
    print("Enter the remote username and password")
    confDict["username"] = input("username: ")
    confDict["password"] = getpass.getpass("password: ")

    # write the config to tmp as a yaml file
    with open("/tmp/remote_setup_cf.yaml", "w") as file:
        confDump = yaml.dump(confDict)
        file.write(confDump)

    return confDict


def connect_ssh(settings: dict):
    username = settings["username"]
    target = settings["target_name"]

    # test the connection first
    if not testNetwork(target):
        print(f"Failed to ping {target}. Make sure the device is online then re-run this configuration")
        exit()

    # handle SSH keys
    sshkeys = glob.glob(os.path.join(HOME_DIR, ".ssh", f"sshkey_{target}"))
    if(len(sshkeys) > 0):
        print(f"\n\nFound existing SSH key for target: {target }")
    else:
        print(f"\n\nConfiguring SSH Keys for {username}@{target}")
        try:
            execLocalScript(os.path.join("config_host", "relationship.bash"), [target, username])
        except CalledProcessError as e:
            if e.returncode != 0 and e.returncode != 1:
                exit()

    # chack that the remote also has internet, this will be used to pull packages
    if not testNetworkRemote("google.com", username, target):
        print("Target does not have internet access. Please make sure it is connected to the internet")
        exit()

    # configure passwordless sudo on target
    remoteExec(f'echo "{username} ALL=(ALL:ALL) NOPASSWD: ALL" | sudo tee /etc/sudoers.d/{username}',
        username, target, root=True, passwd=settings["password"])

    # sync the clocks
    scriptRun = os.path.join("config_target", "date_set.bash")
    execLocalScript(scriptRun, [target, username])

    # make sure the target is arm at least
    _, remoteArch = remoteExecResult("uname -m", username, target, passwd=settings["password"])
    if len(remoteArch) < 0 or not "aarch64" in remoteArch:
        cont = input("The connected system is not aarch64! Continue? y/n")
        if cont.lower() != "y" or cont.lower() != "yes":
            print("Aborting configuration")
            exit()

    # now xfer the script dir
    print("Transferring scripts to remote")
    xferDir(BASE_DIR, username, target, "~")

    # if we have a tarball to transfer, do so
    if(settings["tarball_name"] != None):
        print("Transferring ROS Tar archive, this may take a moment")
        xferSingleFile(settings["tarball_name"], username, target, "~")

def target_configuration(settings: dict):
    username = settings["username"]
    target = settings["target_name"]
    tarball = settings["tarball_name"]

    # install dependencies
    print("\nInstalling dependencies:")
    scriptRun = os.path.join("~", "scripts", "unpack_install", "install_deps.bash")
    remoteExec(f"/bin/bash {scriptRun}", username, target)

    # install pytorch
    print("\nInstalling PyTorch on the target:")
    scriptRun = os.path.join("~", "scripts", "unpack_install", "pytorch_install.bash")
    remoteExec(f"/bin/bash {scriptRun}", username, target)

    if(tarball):
        print("Installing ROS tar")
        scriptRun = os.path.join("~", "scripts", "unpack_install", f"install_tar.bash {ROS_DISTRO} {tarball}")
        remoteExec(f"/bin/bash {scriptRun}", username, target)
    else:
        print("Skipping remote tar install as one was not found locally")

    # configure bashrc
    print("\nSetting up .bashrc")
    scriptRun = os.path.join("~", "scripts", "unpack_install", "setup_bashrc.bash")
    remoteExec(f"/bin/bash {scriptRun}", username, target)

    # configure JetPack settings
    print("\nConfiguring JetPack settings on target:")
    scriptRun = os.path.join("~", "scripts", "config_target", "configure_jetpack.bash")
    remoteExec(f"/bin/bash {scriptRun}", username, target)
      
    

##########################################################################################
### Helper functions below this point used for hashing, user input and shell execution ###
##########################################################################################

def scan_hosts() -> list:
    hosts = []
    # read etc hosts
    with open("/etc/hosts", "r") as data:
        lines = data.readlines()

        # remove commented out lines
        filteredLines = [line.strip() for line in lines if not line.startswith("#") and line.strip() != ""]

        # get the names for each host
        for line in filteredLines:
            hosts.append(line.split("#")[0].split()[1])
        
    return hosts


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
def execute(fullCmd, printOut=False):
    if printOut: print(fullCmd)
    proc = Popen(fullCmd, stdout=PIPE, stderr=PIPE, universal_newlines=True)
    if printOut:
        for line in iter(proc.stdout.readline, ""):
            print(line)
        for errLine in iter(proc.stderr.readline, ""):
            print(errLine)
    proc.stdout.close()
    retCode = proc.wait()
    if retCode:
        raise CalledProcessError(retCode, fullCmd)

def remoteExecResult(cmd, username, address, printOut=False, root=False, passwd=""):
    if root:
        connect = Connection(f"{username}@{address}", config=Config(overrides={"sudo": {"password": passwd}}))
        result = connect.sudo(cmd, hide=(not printOut))
        return (result.exited, str(result).rsplit("\n") if result else [])
    else:
        connect = Connection(f"{username}@{address}")
        result = connect.run(cmd, hide=(not printOut))
        return (result.exited, str(result).rsplit("\n") if result else [])

def remoteExec(cmd, username, address, printOut=False, root=False, passwd=""):
    if root:
        connect = Connection(f"{username}@{address}", config=Config(overrides={"sudo": {"password": passwd}}))
        result = connect.sudo(cmd, hide=(not printOut))
        return result.exited
    else:
        connect = Connection(f"{username}@{address}")
        result = connect.run(cmd, hide=(not printOut))
        return result.exited
    

def testNetwork(pingAddress):
    if call(["ping", "-c", "1", pingAddress], stdout=open(os.devnull, 'wb')) != 0:
        return False
    return True

def testNetworkRemote(pingAddress, username, address):
    if remoteExec(f"ping -c 1 {pingAddress}", username, address) != 0:
        return False
    return True

def execLocalScript(localPath, args):
    scriptPath = os.path.join(os.getcwd(), localPath)
    if not os.path.isfile(scriptPath):
        print(f"Script {scriptPath}, does not exist")
        exit()
    callList = [scriptPath]
    callList.extend(args)
    return execute(callList, printOut=True)

def makeRemoteDir(remoteDir, username, address):
    remoteExec(f"mkdir -p {remoteDir}", username, address, False)

def xferSingleFile(localFile, username, address, destination):
    execute(["rsync", "-vzc", localFile, f"{username}@{address}:{destination}"], False)

def xferDir(localdir, username, address, destination):
    execute(["rsync", "-vrzc", "--delete", "--exclude=**/.git/", "--exclude=**/.vscode/", localdir, f"{username}@{address}:{destination}"], False)

def runRemoteCmd(remoteScript, username, address):
    remoteExec(remoteScript, username, address, True)




# main invoker
if __name__ == "__main__":
    main()