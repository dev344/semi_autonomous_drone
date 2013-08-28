#!/usr/bin/env bash
source /opt/ros/groovy/setup.bash 
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/ros_home
python event_listener.py
