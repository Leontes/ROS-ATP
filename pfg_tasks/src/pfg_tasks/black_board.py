#!/usr/bin/env python

"""
black_board.py - Version 0.7 2015-04-13

Class designed as a intermediate between the htn planner 
and the beaviour tree

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
from geometry_msgs.msg import Pose
from pi_trees_ros.pi_trees_ros import *

class BlackBoard(object):
	""" class BlackBoard

	"""


	def __init__ (self):
		self.task_list = {}
		self.movementTask = False
		self.worldCoords = {}
		self.world = False
		self.worldOperators = {}
		self.routinesList = []


	def setTask(self, action, task):
		""" Creates a new pair task:action in the BlackBoard

		Keyword arguments:
		action -- String indicating the name of the action in the pddl domain
		task -- Executable leaf task of the tree
		"""
		self.task_list.update({action:task})

	def getTask(self, task):
	"""Returns the desired task. False in case that not exists

	Keyword arguments:
	task -- String indicating the task name
	"""
	if(task in self.task_list):
		return self.task_list[task]
	else:
		return False


	def setMovementTask(self, action):
		""" Creates a reference to the movement task of the plan in the BlackBoard

		Keyword arguments:
		task -- String indicating the name of the movement action in the pddl domain
		"""
		self.movementTask = task

	
	def setRoutine(self, routine):
		""" Adds the given routine in the routines list

		Keyword arguments:
		routine -- Exectable Task with the full routine for the robot 
		"""
		self.routinesList.insert(len(self.routinesList), routine)


	def setRobotOrigin(self, place):
		"""Sets the place where the robot starts the execution. Returns false in 
		case of the place hasn't been initializated

		Keyword arguments:
		place -- String indicating the place
		"""
		if place in self.worldCoords:
			self.lastPlace = place
		else:
			return False


	def setCoords(self, entity, x, y):
		"""Creates a new pair entity:Coordinates 

		Keyword arguments:
		entity -- String refered to a place that has representation in the simulator
		x -- Float indicating the value of the x coordinate
		y -- Float indicating the value of the x coordinate
		"""

		#Converts the [x,y] pair to ha Pose data type, for later functionality
		coords = Pose()
		coords.position.x = x
		coords.position.y = y

		coords.orientation.w = 1.0

		self.worldCoords.update({entity:coords})


	def getCoords(self, entity):
		""" Returns the coordinates of a given entity. Returns false if doesn't exists

		Keyword arguments:
		entity -- String indicating the place
		"""
		if entity in self.worldCoords:
			return self.worldCoords[entity]
		else:
			return False