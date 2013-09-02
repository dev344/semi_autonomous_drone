#!/usr/bin/env bash
source /opt/ros/groovy/setup.bash 
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/ros_home
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/semi_autonomous_drone/
gnome-terminal -x python controller.py
python event_listener.py
wait
