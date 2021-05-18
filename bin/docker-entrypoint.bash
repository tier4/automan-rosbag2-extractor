#!/bin/bash
set -e

export ROS_ROOT=/opt/ros/foxy/share/ros
export ROS_PACKAGE_PATH=/opt/ros/foxy/share
#export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/foxy/lib/x86_64-linux-gnu/:/opt/ros/foxy/lib/:/opt/ros/foxy/opt/yaml_cpp_vendor/lib/
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(cat ./ld_path)
export ROS_DISTRO=foxy
#export PYTHONPATH=/opt/ros/foxy/lib/python2.7/dist-packages
export PYTHONPATH=/opt/ros/foxy/lib/python3.8/site-packages:/usr/local/lib/python3.8/dist-packages
export PKG_CONFIG_PATH=/opt/ros/foxy/lib/pkgconfig
export CMAKE_PREFIX_PATH=/opt/ros/foxy
export ROS_ETC_DIR=/opt/ros/foxy/etc/ros

export AMENT_PREFIX_PATH=/opt/ros/foxy

cd /app
exec $@
