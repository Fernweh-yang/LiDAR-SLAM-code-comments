set -e

echo "installing livox_sdk"

cd /opt
git clone https://github.com/Livox-SDK/Livox-SDK.git
cd Livox-SDK
cd build && cmake ..
make
make install

echo "installing livox_sdk DONE"