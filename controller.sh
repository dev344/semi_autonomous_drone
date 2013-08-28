#!/usr/bin/env bash
source /opt/ros/groovy/setup.bash 
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/ros_home
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/ros_home/semi_autonomous_drone/
python event_listener.py
