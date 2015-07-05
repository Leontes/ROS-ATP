#!/usr/bin/env python

"""
core_tasks.py - Version 1.0 2015-04-14

Colection of general purpouse leaf tasks for the behaviour tree 

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

import rospy
import random

from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Twist
from atp_msgs.srv import *

from math import sqrt, pow

from pi_trees_ros.pi_trees_ros import *
from tasks import global_vars



class goToTask(SimpleActionTask):
	"""Class goToTask. When executed the robot moves to a position.

	"""
	def __init__(self, name, coords):
		""" Creates a object of the type goToTask

		Keywords arguments:
		name -- String indicates the name of the task in the tree
		coords -- Pose indicates the coordinates wich should move the robot
		"""
		#Converts the pose to a goal and sends it to ha super class that actually 
		#moves the robot
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = 'map'
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose = coords
		super(goToTask, self).__init__(name, "move_base", MoveBaseAction, 
			goal, reset_after=False, feedback_cb=update_robot_position, result_timeout=60)

	def run(self):
		""" Executes the task. If already executed doesn't nothing

		"""
		return super(goToTask, self).run()


class spinLeftTask(Task):
	"""Class spinLeftTask. When executed the robot pivotes counterclockwise.

	"""
	def __init__(self, name, timer):
		""" Creates a object of the type spinLeftTask

		Keywords arguments:
		name -- String indicates the name of the task in the tree
		timer -- Integer indicates the time in deciseconds
		"""
		super(spinLeftTask, self).__init__(name, children = None)
		self.name = name
		self.timer = timer
		self.finished = False

		self.cmd_vel_pub = rospy.Publisher('cmd_vel',Twist, queue_size = 10)
		self.cmd_vel_msg = Twist()
		self.cmd_vel_msg.linear.x = 0
		self.cmd_vel_msg.angular.z = 1.5


	def run(self):
		""" Executes the task. If already executed doesn't nothing

		"""
		if self.finished:
			return TaskStatus.SUCCESS
		else:

			self.cmd_vel_pub.publish(self.cmd_vel_msg)
			self.timer -= 1
			rospy.sleep(0.1)
			if(self.timer == 0):
				self.finished = True
				self.cmd_vel_pub.publish(Twist())
			return TaskStatus.RUNNING
			


class spinRightTask(Task):
	"""Class spinRightTask. When executed the robot pivotes clockwise.
	
	"""
	def __init__(self, name, timer):
		""" Creates a object of the type spinRightTask

		Keywords arguments:
		name -- String indicates the name of the task in the tree
		timer -- Integer indicates the time in deciseconds
		"""
		super(spinRightTask, self).__init__(name, children = None)
		self.name = name
		self.timer = timer
		self.finished = False

		self.cmd_vel_pub = rospy.Publisher('cmd_vel',Twist, queue_size = 10)
		self.cmd_vel_msg = Twist()
		self.cmd_vel_msg.linear.x = 0
		self.cmd_vel_msg.angular.z = -1.5


	def run(self):
		""" Executes the task. If already executed doesn't nothing

		"""
		if self.finished:
			return TaskStatus.SUCCESS
		else:
			self.cmd_vel_pub.publish(self.cmd_vel_msg)
			self.timer -= 1
			rospy.sleep(0.1)
			if(self.timer == 0):
				self.finished = True
				self.cmd_vel_pub.publish(Twist())
			return TaskStatus.RUNNING


class sleepTask(Task):
	"""Class sleepTask. When executed the robot doesn't nothing for a while
	
	"""
	def __init__(self, name, timer):
		""" Creates a object of the type sleepTask

		Keywords arguments:
		name -- String indicates the name of the task in the tree
		timer -- Integer indicates the time in deciseconds
		"""
		super(sleepTask, self).__init__(name, children = None)
		self.name = name
		self.sleep = timer
		self.finished = False


	def run(self):
		""" Executes the task. If already executed doesn't nothing

		"""
		if self.sleep <= 0:
			self.finished = True

		if self.finished:
			return TaskStatus.SUCCESS
		else:
			self.sleep -= 1
			rospy.sleep(0.1)
			return TaskStatus.RUNNING


class checkDoneTask(Task):
	""" Class checkDoneTask. When executed checks if the task has been executed

	"""
	def __init__(self, name, index, state):
		""" Creates a object of the type checkDoneTask

		Keywords arguments:
		name -- String indicates the name of the task in the tree
		index -- Integer indicates the task to be checked
		"""
		super(checkDoneTask, self).__init__(name, children = None)
		self.index = index
		self.state = state

	def run(self):
		""" Executes the task. The task checks the task in the black board, 
		if its have been executed returns Success, else returns FAILURE

		"""
		
		if global_vars.black_board.checkDone(self.index) == True:
			return TaskStatus.SUCCESS
		elif global_vars.black_board.rePlanNeeded():
			return TaskStatus.SUCCESS
		elif compareState(self.state, global_vars.black_board.getWorld()):
			global_vars.black_board.setReplan(True)
			return TaskStatus.SUCCESS
		else:
			return TaskStatus.FAILURE


class setDoneTask(Task):
	""" Class checkDoneTask. When executed sets the task as executed
	in the black board

	"""
	def __init__(self, name, index, function, args):
		""" Creates a object of the type checkDoneTask

		Keywords arguments:
		name -- String indicates the name of the task in the tree
		index -- Integer indicates the task to be seted
		"""
		super(setDoneTask, self).__init__(name, children = None)
		self.index = index
		self.operator = function
		self.arguments = args

	def run(self):
		""" Executes the task. The task checks the task in the black board, 
		if its have been executed returns Success,
		else sets it has executed and return Success

		"""
		if global_vars.black_board.checkDone(self.index) == True:
			return TaskStatus.SUCCESS
		else:
			fallo = random.random()*100.0
			if fallo < global_vars.black_board.failChance:
				world = global_vars.black_board.getWorld()
				world = self.operator(world, *self.arguments)
				
				global_vars.black_board.setWorld(world)
				
			global_vars.black_board.setDone(self.index)
			return TaskStatus.SUCCESS
		

class checkLocationTask(Task):
	"""Class checkLocationTask. When executed checks the position in the simulator
	and compare it with a predefined position

	"""
	def __init__(self, place):
		""" Creates a object of the type checkLocationTask

		Keywords arguments:
		place -- String indicates the name of the task in the tree
		timer -- Integer indicates the time in deciseconds
		"""
		name = "CHECKLOCATION: " + place
		super(checkLocationTask, self).__init__(name)    
		self.name = name
		self.place = place

	def run(self):
		""" Executes the task. The task computes the euclidean distance between the 
		robot and the waypoint

		"""
		waypoint = global_vars.black_board.getCoords(self.place).position
		robot = global_vars.black_board.getCoords("robot").position

		distance = sqrt(pow((waypoint.x - robot.x),2) + pow((waypoint.y - robot.y),2)
			+ pow((waypoint.z - robot.z),2))

		if distance < 0.1:
			status = TaskStatus.SUCCESS
		else:
			status = TaskStatus.FAILURE

		return status


def update_robot_position(msg):
	""" Auxiliar function. Updates the current robot position in the black board.

	Keywords arguments:
	msg -- Pose current position of the robot.
	"""
	global_vars.black_board.setCoords("robot", msg.base_position.pose.position.x,
		msg.base_position.pose.position.y)


def check_battery(msg):
	"""Auxiliar function. Checks the mensage given and compares with
	the minimun battery level set.

	Keywords arguments:
	msg -- Float indicates the current battery level.
	"""
	if msg.data is None:
		return TaskStatus.RUNNING
	else:
		if msg.data < global_vars.low_battery_threshold:
			rospy.loginfo("LOW BATTERY - level: " + str(int(msg.data)))
			return TaskStatus.FAILURE
		else:
			return TaskStatus.SUCCESS
    
	
def recharge_cb(result):
	"""Auxiliar function. Notifies when the robot is charged.

	"""
	rospy.loginfo("BATTERY CHARGED!")



def compareState(state1, state2):
	if state1 == False:
		return False
	elif state2 == False:
		return False
	else:
		elements1 = vars(state1)
		elements2 = vars(state2)
		return (elements1 != elements2)