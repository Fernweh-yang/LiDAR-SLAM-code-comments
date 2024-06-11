set -e

echo "installing livox_ros"

cd ~
echo "$PWD"
git clone https://github.com/Livox-SDK/livox_ros_driver.git ws_livox/src
cd ws_livox
catkin_make
echo "source $PWD/devel/setup.bash" >> ~/.bashrc

echo "installing livox_sdk DONE"