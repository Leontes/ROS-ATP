#!/usr/bin/env python

"""
core_tasks.py - Version 1.0 2015-04-14

Colection of general purpouse leaf tasks for the beaviour tree 

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

from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Twist
from pfg_msgs.srv import *

from math import sqrt, pow

from pi_trees_ros.pi_trees_ros import *
from pfg_tasks import global_vars



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
		super(goToTask, self).__init__(name, "move_base", MoveBaseAction, goal, reset_after=False, feedback_cb=update_robot_position)

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
		super(pickUpTask, self).__init__(name, children = None)
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
		super(putDownTask, self).__init__(name, children = None)
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
			rospy.sleep(1)
			return TaskStatus.RUNNING




class checkLocationTask(Task):
	"""Class checkLocationTask. When executed checks the position in the simulator
	and compare it with a predefined position
	
	"""
    def __init__(self, place):
    	""" Creates a object of the type checkLocationTask

		Keywords arguments:
		name -- String indicates the name of the task in the tree
		timer -- Integer indicates the time in deciseconds
		"""
        name = "CHECKLOCATION: " + name
        super(CheckLocation, self).__init__(name)    
        self.name = name
        self.place = place

    def run(self):
    	""" Executes the task. The task computes the euclidean distance between the 
    	robot and the waypoint

		"""

        waypoint = global_vars.black_board.getCoords(self.place).position
        robot = global_vars.black_board.getCoords("robot").position
        
        distance = sqrt(pow((waypoint.x - robot.x),2) + pow((waypoint.y - robot.y),2)
        	+ pow((waypoint.z - robot.z),2)
                                
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