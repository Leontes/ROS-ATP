#!/usr/bin/env python


import rospy
from robotTasks import *
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Pose


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


	"""
	Tree generator
	"""


	def makeRutines(self):
		rutines = Sequence("routines")

		for i in range(len(self.routinesList)):
			rutines.add_child(self.routinesList[i])

		return rutines



	def makeTree(self, plan):

		tree = Sequence("Tree")

		tree.add_child(self.makeRutines())

		execPlan = Sequence("execPlan")

		lastPlace = ""

		for i in range(len(plan)):
			if plan[i][0] == self.movementTask:
				coord = self.getCoords(plan[i][1])
				execPlan.add_child(goToTask("MoveToTask: " + plan[i][1], coord))
				lastPlace = plan[i][1]
			
			else:
				task = self.getTask(plan[i][0])
				if task != False:
					
					coords = black_board.getCoords(lastPlace)
					moveToLasPositionTask = goToTask("MoveToTaskLastPosition: " + lastPlace, coords)
					checkLocationTask = CheckLocation(lastPlace)

					NavigationTask = Selector("NavRoutine", [checkLocationTask, moveToLasPositionTask])

					execPlan.add_child(Sequence("Task "+ plan[i][0], [NavigationTask, task]))
		
		tree.add_child(execPlan)


		return tree




black_board = BlackBoard()
