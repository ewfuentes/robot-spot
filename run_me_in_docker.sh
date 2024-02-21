#! /bin/env bash

CURDIR=$(pwd)

rosdep install brm_evanescence
cd $(dirname $0)/catkin_ws/src/spot_ros
rosdep install --from-paths spot_cam spot_description spot_driver \
    spot_msgs spot_viz spot_wrapper --ignore-src -y
cd spot_wrapper
pip install .

cd $CURDIR
