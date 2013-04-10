#!/usr/bin/env bash
source /opt/ros/groovy/setup.bash 
rostopic pub -1 /ardrone/takeoff std_msgs/Empty
