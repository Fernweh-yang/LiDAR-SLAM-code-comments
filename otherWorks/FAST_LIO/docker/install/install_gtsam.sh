#!/usr/bin/env bash

set -e

echo "Installing gtsam..."


cd /opt
git clone https://github.com/borglab/gtsam.git --depth 1 --branch 4.0.0-alpha2
cd gtsam
mkdir build && cd build
cmake .. -DGTSAM_USE_SYSTEM_EIGEN=ON \
         -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
         -DCMAKE_BUILD_TYPE=Release \
         -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc)
make install -j$(nproc)
rm -rf /opt/gtsam

echo "Installing gtsam... DONE"
