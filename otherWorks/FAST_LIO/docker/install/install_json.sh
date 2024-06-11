#!/usr/bin/env bash

set -e

echo "Installing nlohmann/json..."

cd /opt
git clone https://github.com/nlohmann/json.git --depth 1 --branch v3.11.3
cd json
mkdir build && cd build
cmake .. -DJSON_BuildTests=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc)
make install -j$(nproc)
rm -rf /opt/json

echo "Installing nlohmann/json... DONE"
