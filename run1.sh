#!/bin/bash
set -e
source devel/setup.bash
catkin_make
roslaunch mr_control control.launch

