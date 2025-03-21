#!/bin/bash

echo "Allowing docker to run emulated containers"

# Install the qemu packages
sudo apt-get install qemu binfmt-support qemu-user-static

# This step will execute the registering scripts
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes

echo "Docker can now emulate ARM64 containers"