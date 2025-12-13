#!/bin/bash
cd smart_pick_and_place_ws
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
roslaunch pkg_launch bringup.launch