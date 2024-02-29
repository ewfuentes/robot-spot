#! /bin/env bash

CURDIR=$(pwd)

pip install --force-reinstall beacon_sim*.whl
rosdep install brm_evanescence --ignore-src -y
cd $(dirname $0)/catkin_ws/src/spot_ros
rosdep install --from-paths spot_cam spot_description spot_driver \
    spot_msgs spot_viz spot_wrapper --ignore-src -y
cd spot_wrapper
pip install .

cd $CURDIR
