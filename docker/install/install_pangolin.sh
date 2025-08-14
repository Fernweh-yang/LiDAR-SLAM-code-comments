#!/usr/bin/env bash

set -e

echo "Installing Pangolin..."

cd /opt
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git --depth 1
cd Pangolin
apt update
apt install python3-pip
pip3 install setuptools wheel
cmake -B build -G Ninja -DPYTHON_EXECUTABLE=/usr/bin/python3.8
cmake --build build
rm -rf /opt/Pangolin

echo "Installing Pangolin... DONE"