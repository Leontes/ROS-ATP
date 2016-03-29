#!/bin/bash

chmod 777 src/ROS-ATP/atp_utils/nodes/*.py
chmod 777 src/ROS-ATP/tasks_exec/nodes/*.py
chmod 777 src/ROS-ATP/atp_utils/cfg/BatterySimulator.cfg

apt_get update -y
apt-get install -y ros-indigo-arbotix ros-indigo-rviz
apt-get upgrade -y --force-yes

source /opt/ros/indigo/setup.bash
catkin_make
source devel/setup.bash
