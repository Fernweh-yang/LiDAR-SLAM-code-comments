#!/usr/bin/env bash

set -e

echo "Installing ceres..."

cd /opt
git clone https://github.com/ceres-solver/ceres-solver.git --depth 1 --branch 2.2.0
cd ceres-solver
mkdir build && cd build
cmake .. -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc)
make install -j$(nproc)
rm -rf /opt/ceres-solver

echo "Installing ceres... DONE"
