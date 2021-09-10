#!/bin/bash
set -e

export ROS_ROOT=/opt/ros/galactic/share/ros
export ROS_PACKAGE_PATH=/opt/ros/galactic/share
#export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/galactic/lib/x86_64-linux-gnu/:/opt/ros/galactic/lib/:/opt/ros/galactic/opt/yaml_cpp_vendor/lib/
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(cat ./ld_path)
export ROS_DISTRO=galactic
export PYTHONPATH=/opt/ros/galactic/lib/python3.8/site-packages:/usr/local/lib/python3.8/dist-packages
export PKG_CONFIG_PATH=/opt/ros/galactic/lib/pkgconfig
export CMAKE_PREFIX_PATH=/opt/ros/galactic
export ROS_ETC_DIR=/opt/ros/galactic/etc/ros

export AMENT_PREFIX_PATH=/opt/ros/galactic

cd /app
exec $@
