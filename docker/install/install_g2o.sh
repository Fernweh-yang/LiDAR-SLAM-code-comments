#!/usr/bin/env bash

set -e

echo "Installing g2o..."

cd /opt
git clone https://github.com/RainerKuemmerle/g2o.git --depth 1
cd g2o
mkdir build 
cd build
cmake ..
sudo make install -j$(nproc)
rm -rf /opt/g2o

echo "Installing g2o... DONE"