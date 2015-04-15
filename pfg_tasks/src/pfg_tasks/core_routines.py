#!/usr/bin/env python

"""
core_routines.py - Version 1.0 2015-04-14

Predefined routines for the robot

Copyright (c) 2015 Jose Angel Segura Muros.  All rights reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

"""


from pi_trees_ros.pi_trees_ros import *
from std_msgs.msg import Float32
from pfg_tasks.core_tasks import *
from pfg_tasks import global_vars

def addCoreRoutines():
	"""Adds the programmed routines to the black board

	"""
	batteryRoutine()


def batteryRoutine():
	""" Creates a routine to the robot thats checks the battery and 
	sends the robot to the dock station for charging purpouses

	"""

	# The "get Charged" routine
	getChargedTask = Selector("getChargedTask")

	# Add the check battery condition
	checkBatteryTask = MonitorTask("checkBattery", "battery_level", Float32, check_battery)


	# Add the recharge task
	chargeRobotTask = ServiceTask("chargeRobot", "battery_simulator/set_battery_level", SetBatteryLevel, 100, result_cb=recharge_cb)

	# Add the movement routine to the dock
	coords = global_vars.black_board.getCoords('dock')
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose = coords
	
	moveToDockTask = SimpleActionTask("MoveToDock", "move_base", MoveBaseAction, goal, reset_after=True,  feedback_cb=update_robot_position)
	checkLocation = checkLocationTask("dock")

	NavigationTask = Selector("Nav", [checkLocation, moveToDockTask] )

	# Build the recharge sequence using inline syntax
	rechargeTask = Sequence("recharge", [NavigationTask, chargeRobotTask])



	# Add the check battery and recharge tasks to the stay healthy selector
	getChargedTask.add_child(checkBatteryTask)
	getChargedTask.add_child(rechargeTask)

	#Add the routine to the black board
	global_vars.black_board.setRoutine(getChargedTask)