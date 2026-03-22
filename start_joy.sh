#!/bin/bash
source /opt/ros/jazzy/setup.bash
source ~/slvrov_ros/install/setup.bash

gnome-terminal -- bash -c "ros2 run joy joy_node --ros-args -r /joy:=/joy_left -p device_id:=0; bash"
gnome-terminal -- bash -c "ros2 run joy joy_node --ros-args -r /joy:=/joy_right -p device_id:=1; bash"
