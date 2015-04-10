#!/usr/bin/env python


import rospy
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Pose
from pi_trees_ros.pi_trees_ros import *

class BlackBoard(object):


	def __init__ (self):
		self.task_list = {}

		self.movementTask = False

		self.worldCoords = {}

		self.world = False

		self.worldOperators = {}

		self.routinesList = []



	"""
	Task related functions
	"""

	def setTask(self, task, action):
		"""
		Creates a new pair (task:action) in the BlackBoard
		"""
		self.task_list.update({task:action})


	def setMovementTask(self, task):
		"""
		Creates a reference to the movement task of the plan in the BlackBoard
		"""
		self.movementTask = task

	def setRoutine(self, routine):
		self.routinesList.insert(len(self.routinesList), routine)


	def getTask(self, task):
		"""
		Get info about a desired task
		"""
		if(task in self.task_list):
			return self.task_list[task]
		else:
			return False


	def getAllTasks(self):
		"""
		Get al task in task_list
		"""
		return self.task_list


	def printTasks(self):
		"""
		Print all the task_list
		"""
		print(self.getAllTasks())


	"""
	Coordinates related functions
	"""

	def setRobotOrigin(self, place):
		if place in self.worldCoords:
			self.lastPlace = place
		else:
			return False

	def setCoords(self, entity, x, y):
		"""
		Creates a new pair entity:Coordinates
		"""
		coords = Pose()
		coords.position.x = x
		coords.position.y = y

		coords.orientation.w = 1.0

		self.worldCoords.update({entity:coords})


	def getCoords(self, entity):
		"""
		Returns the coordinates of a given entity
		"""
		if entity in self.worldCoords:
			return self.worldCoords[entity]
		else:
			return False


	"""
	World related functions
	"""

	def setWorld(self, state):
		self.world = state


	def setWorldOperators(self, operators):
		self.worldOperators = operators


	def getWorld(self):
		return self.world


	def getWorldOperator(self, operator):
		return self.worldOperators[operator]
