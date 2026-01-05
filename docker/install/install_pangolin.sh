#!/usr/bin/env bash

set -e

echo "Installing Pangolin..."

cd /opt
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git --depth 1
cd Pangolin
apt update
apt install python3-pip -y
pip3 install setuptools wheel
cmake -B build -G Ninja -DPYTHON_EXECUTABLE=/usr/bin/python3.8 -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-Wno-deprecated-copy"
cmake --build build -j$(nproc)
sudo cmake --install build
sudo ldconfig
rm -rf /opt/Pangolin

echo "Installing Pangolin... DONE"