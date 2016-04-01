#!/bin/bash

chmod 777 src/ROS-ATP/atp_utils/nodes/*.py
chmod 777 src/ROS-ATP/tasks_exec/nodes/*.py
chmod 777 src/ROS-ATP/atp_utils/cfg/BatterySimulator.cfg

apt-get update -y --force-yes
apt-get install -y --force-yes ros-indigo-arbotix ros-indigo-rviz
apt-get upgrade -y --force-yes

source /opt/ros/indigo/setup.bash
catkin_make
source devel/setup.bash
