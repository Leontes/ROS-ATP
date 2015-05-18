#!/usr/bin/python

"""
domain.py - Version 1.0 2015-05-14

Class designed to codify a pddl domain

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

from pyhop import hop

class Domain(object):
	""" class Domain

	"""
	def __init__(self):
		self.state = hop.State("Initial State")
		self.objList = {}
		self.taskList = []
		self.methodList = {}


	def setName(self, name):
		""" Set the domain name

		Keyword arguments:
		name -- String indicating the name of the pddl domain
		"""
		self.name = name


	def checkDomainName(self, name):
		""" Checks the domain name

		Keyword arguments:
		name -- String indicating the name of the pddl domain
		"""
		if self.name != name:
			raise Exception("Domain names not match in the domain and problem files")

	def setProblemName(self, problem):
		""" Set the problem name

		Keyword arguments:
		name -- String indicating the name of the pddl problem
		"""
		self.problemName = problem


	def setRequirements(self, requirements):
		""" Set the requirements of the domain

		Keyword arguments:
		requirements -- Boolean array indicating the requirements of the domain
		"""

		self.strips = False
		self.equility = False
		self.typing = False
		self.adl = False 

		for i in range(len(requirements)):
			if requirements[i] == ":strips":
				self.strips = True
			if requirements[i] == ":equility":
				self.equility = True
			if requirements[i] == ":typing":
				self.typing = True
			if requirements[i] == ":adl":
				self.adl = True

	def setTypes(self, types):
		""" Set the object types of the domain

		Keyword arguments:
		types -- token array indicating the types of the domain
		"""
		self.types = []
		for tp in types:
			self.types.append(tp.upper())

	def setPredicates(self, predicates):
		""" Set the predicates of the domain

		Keyword arguments:
		predicates -- token array indicating the predicates
		"""
		self.objTypes = {}
		for i in range(len (predicates)):
			objAux = []
			for j in range(len(predicates[i])):
				if predicates[i][j] == "-":
					if not (predicates[i][j+1].upper() in self.types):
						raise Exception(predicates[i][j+1].upper() + " type not defined")
					else:
						objAux.append(predicates[i][j+1])
			self.objTypes.update({predicates[i][0]:objAux})
			setattr(self.state, predicates[i][0].upper(), "__NON_DEFINED__")

	def setObjects(self, objList):
		""" Set the objects of the domain

		Keyword arguments:
		types -- token array indicating the objects of the domain
		"""
		for i in range(len(objList)):
			if objList[i] == "-":
				i += 2
			j = i
			while j < len(objList):
				if objList[j] != "-":
					j += 1
				else:
					self.objList.update({objList[i]:objList[j+1]})
					break


	def setGoals(self, goals):
		""" Set the goals of the domain

		Keyword arguments:
		goals -- formated token array indicating the goals of the problem domain
		"""
		self.goals  = goals


	def getGoals(self):
		""" Get the goals of the domain

		"""
		return self.goals


	def initState(self, initList):
		""" Sets the initial state of the domain

		Keyword arguments:
		initList -- token stream with the predicates of the initial state
		"""
		for i in range(len(initList)):
			if getattr(self.state, initList[i][0].upper(), False) == False:
				raise Exception(predicates[i][0].upper() + " predicate not defined")
			elif getattr(self.state, initList[i][0].upper(), False) == "__NON_DEFINED__":
				if len(initList[i][1:]) != 0:
					setattr(self.state, initList[i][0].upper(), [initList[i][1:]])
				else: 
					setattr(self.state, initList[i][0].upper(), True)
			else:
				aux = getattr(self.state, initList[i][0].upper(), False)
				aux.append(initList[i][1:])
				setattr(self.state, initList[i][0].upper(), aux)


	def setTasks(self, task):
		""" Set the tasks of the domain

		Keyword arguments:
		task -- Python executable function that implements a pyhop task
		"""
		self.taskList.append(task)


	def setMethods(self, methodName, method):
		""" Set the methods of the domain

		Keyword arguments:
		methodName -- String, name of the method
		method -- Python executable function that implements a pyhop method
		"""
		self.methodList.update({methodName:method})


	def printState(self):
		""" Prints the initial state

		"""
		hop.print_state(self.state)