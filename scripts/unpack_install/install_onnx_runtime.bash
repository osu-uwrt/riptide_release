#!/bin/bash

# https://forums.developer.nvidia.com/t/issue-using-onnxruntime-with-cudaexecutionprovider-on-orin/219457/8

sudo apt install -y --no-install-recommends build-essential software-properties-common libopenblas-dev libpython3.8-dev python3-pip python3-dev python3-setuptools python3-wheel
sudo apt install -y protobuf-compiler libprotobuf-dev openssl libssl-dev libcurl4-openssl-dev autoconf bc g++-8 gcc-8 clang-8 lld-8 gettext-base gfortran-8 iputils-ping libbz2-dev libc++-dev libcgal-dev libffi-dev libfreetype6-dev libhdf5-dev libjpeg-dev liblzma-dev libncurses5-dev libncursesw5-dev libpng-dev libreadline-dev libssl-dev libsqlite3-dev libxml2-dev libxslt-dev locales moreutils openssl python-openssl rsync scons

cd ~
wget http://www.cmake.org/files/v3.18/cmake-3.18.0.tar.gz
tar xpvf cmake-3.18.0.tar.gz cmake-3.18.0/
cd cmake-3.18.0/
./bootstrap --system-curl
make -j8
echo 'export PATH=~/cmake-3.18.0/bin/:$PATH' >> ~/.bashrc
export PATH=~/cmake-3.18.0/bin/:$PATH

cd /usr/local/include
sudo mkdir onnxruntime
sudo chown -R $USER onnxruntime

# git clone --recursive -b rel-1.13.1 https://github.com/microsoft/onnxruntime
cd onnxruntime/

./build.sh --config RelWithDebInfo --update --build --parallel \
 --use_tensorrt --cuda_home /usr/local/cuda --cudnn_home /usr/lib/aarch64-linux-gnu \
 --tensorrt_home /usr/lib/aarch64-linux-gnu --build_shared_lib

# sudo pip3 install build/Linux/Release/dist/onnxruntime_gpu-1.12.0-cp38-cp38-linux_aarch64.whl
sudo ln -snf $(pwd)/build/Linux/RelWithDebInfo/*.so /usr/lib/ 
sudo ln -snf /usr/lib/libonnxruntime.so /usr/lib/libonnxruntime.so.1.13.1
