export ROSMAV=`pwd`
export ROS_PACKAGE_PATH=$ROSMAV:$ROSMAV/cmvision:$ROSMAV/brown-ros-pkg-read-only/experimental:$ROSMAV/stacks:$ROS_PACKAGE_PATH
pushd brown-ros-pkg-read-only/experimental/ardrone_brown
echo "Building ardrone_brown"
rm CMakeCache.txt
cmake .
./build_sdk.sh
popd
pushd cmvision
echo "building cmvision"
rm CMakeCache.txt
rosmake cmvision
popd
echo export ROSMAV=`pwd` >> ~/.bashrc
echo export ROS_PACKAGE_PATH=$ROSMAV:$ROSMAV/cmvision:$ROSMAV/brown-ros-pkg-read-only/experimental:$ROSMAV/stacks:$ROS_PACKAGE_PATH >> ~/.bashrc
echo export PATH=$PATH:$ROSMAV/brown-ros-pkg-read-only/experimental/drone_teleop/bin >> ~/.bashrc
