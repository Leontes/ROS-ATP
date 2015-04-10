cmake_minimum_required(VERSION 2.8.3)

project(pfg_utils)

find_package(catkin REQUIRED COMPONENTS	dynamic_reconfigure)


generate_dynamic_reconfigure_options(
	cfg/BatterySimulator.cfg
	)

catkin_package(DEPENDS CATKIN DEPENDS dynamic_reconfigure message_runtime)
