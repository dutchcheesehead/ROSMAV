pushd brown-ros-pkg-read-only/experimental/ardrone_brown
rm CMakeCache.txt
cmake .
./build_sdk.sh
popd
pushd cmvision
rm CMakeCache.txt
cmake .
popd
echo export ROSMAV=`pwd` >> ~/.bashrc
echo export ROS_PACKAGE_PATH=$ROSMAV:$ROSMAV/cmvision:$ROSMAV/brown-ros-pkg-read-only/experimental:$ROSMAV/stacks:$ROS_PACKAGE_PATH >> ~/.bashrc

