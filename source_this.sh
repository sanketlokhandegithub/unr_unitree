#!/bin/sh
source /opt/ros/melodic/setup.bash
export ROS_PACKAGE_PATH=~/testa1_ws:${ROS_PACKAGE_PATH}
export GAZEBO_PLUGIN_PATH=~/testa1_ws/devel/lib:${GAZEBO_PLUGIN_PATH}
export LD_LIBRARY_PATH=~/testa1_ws/devel/lib:/usr/local/lib:${LD_LIBRARY_PATH}
# 3_1, 3_2
export UNITREE_SDK_VERSION=3_2
export UNITREE_LEGGED_SDK_PATH=~/testa1_ws/src/unitree_legged_sdk
# amd64, arm32, arm64
export UNITREE_PLATFORM="amd64"


source devel/setup.bash
