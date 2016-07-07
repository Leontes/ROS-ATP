#!/bin/bash

chmod 777 src/ROS-ATP/atp_utils/nodes/*.py
chmod 777 src/ROS-ATP/tasks_exec/nodes/*.py
chmod 777 src/ROS-ATP/atp_utils/cfg/BatterySimulator.cfg

apt-get update -y --force-yes
apt-get install -y --force-yes ros-jade-arbotix ros-jade-rviz ros-jade-fake-localization ros-jade-map-server
apt-get upgrade -y --force-yes

source /opt/ros/jade/setup.bash
catkin_make
source devel/setup.bash
