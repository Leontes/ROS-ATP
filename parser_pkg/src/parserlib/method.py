#!/usr/bin/python

"""
method.py - Version 1.0 2015-05-14

Executable class thats implements a generic pyhop method 

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


class Method(object):
	""" class Method

	"""


	def __init__(self, name, domain, parameters, cases):
		""" Construct a object of the class Method

		Keyword arguments:
		name -- String, name of the method
		domain -- Domain object asociated to the method
		parameters -- Token stream with the parameters of the method
		cases -- Token stream with the cases of the method
		"""
		self.__name__ = name.upper()
		self.domain = domain
		self.setParameters(parameters)
		self.cases = self.setCases(cases)


	def  __call__(self, State, *args):
		""" Executes the Method object

		Keyword arguments:
		State -- Pyhop State object 
		args -- Arguments of the execution
		"""
		if(self.checkArgs(State, args) == True):
			return self.execCases(State, args) 
		return False 


	def setParameters(self, parameters):
		""" Set the parameters of the method

		Keyword arguments:
		parameters -- Token stream with the info of the parameters
		"""
		self.params = []
		for i in range(len(parameters)):
			if parameters[i] == "-":
				if parameters[i+1].upper() in self.domain.types:
					self.params.append([parameters[i-1], parameters[i+1].upper()])
				else:
					raise Exception ("Type " + parameters[i+1].upper() + " not defined")


	def checkArgs(self, state, *args):
		""" Check the arguments used to call the method

		Keyword arguments:
		state -- Pyhop State object
		args -- Arguments of the execution
		"""
		self.linkedParams = {}
		for arg in args:
			for i in range(len(arg)):
				if self.domain.objList[arg[i]].upper() != self.params[i][1]:
					return False
				else:
					self.linkedParams.update({self.params[i][0]:arg[i]})
		for k in self.domain.objList.keys():
			self.linkedParams.update({k:k})
			
		return True


	def setCases(self, cases):
		""" Sets the cases of the method

		Keyword arguments:
		cases -- Token stream with the cases info
		"""
		auxDict = {}
		for i in range(len(cases)):
			auxDict.update({cases[i][1]: cases[i][2:]})

		return auxDict


	def execCases(self, state, args):
		""" Executes the cases of the method

		Keyword arguments:
		state -- Pyhop State object
		args -- Arguments of the execution
		"""
		for i in self.cases:
			prec = self.cases[i][1]
			tasks = self.cases[i][3]

			self.unifiedParams = {}
			self.formatPreconditions(state, prec)
			if self.calculateVars(self.unifiedParams.keys(), state, prec) == True:
				return self.formatTasks(tasks)
			else:
				self.cleanLinkedParams()

		return False




	def checkPreconditions(self, State, preconditions):
		""" Checks the given preconditions with que state

		Keyword arguments:
		State -- Pyhop State object
		preconditions -- Token stream with the preconditions list
		"""
		if preconditions[0] == "and":
			return self.checkPreconditionsAnd(State, preconditions[1:])
		elif preconditions[0] == "or":
			return self.checkPreconditionsOr(State, preconditions[1:])
		elif preconditions[0] == "not":
			return self.checkPreconditionsNot(State, preconditions[1:])
		prec = getattr(State, preconditions[0].upper(), "Predicate_not_defined")
		if prec == "Predicate_not_defined":
			raise Exception("Token " + str(preconditions[0]).upper() + " not defined")
		else:
			aux = []
			for i in range(1, len(preconditions)):
				aux.append(self.linkedParams[preconditions[i]])
			if len(aux) != 0:
				match = False
				for j in range(len(prec)):
					if aux == prec[j]:
						match = True

				return match
			else:
				if prec == "__NON_DEFINED__" or prec == False:
					return False
				else:
					return True


	def checkPreconditionsNot(self, State, precondition):
		""" Checks the given NOT precondition with que state

		Keyword arguments:
		State -- Pyhop State object
		preconditions -- Token stream with the preconditions list
		"""
		preconditions = precondition[0]
		if preconditions[0] == "and":
			return not self.checkPreconditionsAnd(State, preconditions[1:])
		elif preconditions[0] == "or":
			return not self.checkPreconditionsOr(State, preconditions[1:])
		elif preconditions[0] == "not":
			return not self.checkPreconditionsNot(State, preconditions[1:])
		prec = getattr(State, preconditions[0].upper(), "Predicate_not_defined")
		if prec == "Predicate_not_defined":
			raise Exception("Token " + str(preconditions[0]).upper() + " not defined")
		else:
			aux = []
			for i in range(1, len(preconditions)):
				aux.append(self.linkedParams[preconditions[i]])
			if len(aux) != 0:
				match = False
				for j in range(len(prec)):
					if aux == prec[j]:
						match = True

				return not match
			else:
				if prec == "__NON_DEFINED__" or prec == False:
					return True
				else:
					return False


	def checkPreconditionsAnd(self, State, preconditionAuxList):
		""" Checks the given AND precondition with que state

		Keyword arguments:
		State -- Pyhop State object
		preconditionAuxList -- Token stream with the preconditions list
		"""
		#For all preconditions
		for i in range(len(preconditionAuxList)):
			#Take 1
			precondition = preconditionAuxList[i]
			#If theres a list with an OR statement
			if precondition[0] == "or":
				#Check the OR sublist
				if self.checkPreconditionsOr(State, precondition[1:]) == False:
					return False
			elif precondition[0] == "not":
				if self.checkPreconditionsNot(State, precondition[1:]) == False:
					return False
 			else:
 				#Check with the state
 				predicate = getattr(State, precondition[0].upper(), "Predicate_not_defined")
 				#Lexic control
 				if predicate == "Predicate_not_defined":
 					raise Exception(precondition[0].upper() + " not defined")
 				else:
 					#Auxiliar list with the parameters
 					aux = []
 					for j in range(1, len(precondition)):
 						#Filling the auxiliar list...
 						aux.append(self.linkedParams[precondition[j]])
 					match = False
 					if len(aux) != 0:
	 					#Check with the state
	 					for j in range(len(predicate)):
	 						if aux == predicate[j]:
	 							match = True
	 					#if theres no match finish(its an AND we only need 1 missmatch to return False)
	 				else:
						if predicate == True:
							match = True		

 					if match == False:
 						return False

 		return True


	def checkPreconditionsOr(self, State, preconditionAuxList):
		""" Checks the given OR precondition with que state

		Keyword arguments:
		State -- Pyhop State object
		preconditionAuxList -- Token stream with the preconditions list
		"""
		if self.domain.adl == True:
			#For all preconditions
			for i in range(len(preconditionAuxList)):
				#Take 1
				precondition = preconditionAuxList[i]
				#If theres a list with an OR statement
				if precondition[0] == "and":
					#Check the OR sublist
					if self.checkPreconditionsAnd(State, precondition[1:]) == True:
						return True
				elif precondition[0] == "not":
					if self.checkPreconditionsNot(State, precondition[1:]) == True:
						return True
	 			else:
	 				#Check with the state
	 				predicate = getattr(State, precondition[0].upper(), "Predicate_not_defined")
	 				#Lexic control
	 				if predicate == "Predicate_not_defined":
	 					raise Exception(precondition[0].upper() + " not defined")
	 				else:
	 					#Auxiliar list with the parameters
	 					aux = []
	 					for j in range(1, len(precondition)):
	 						#Filling the auxiliar list...
	 						aux.append(self.linkedParams[precondition[j]])
	 					match = False
	 					if len(aux) != 0:
		 					#Check with the state
		 					for j in range(len(predicate)):
		 						if aux == predicate[j]:
		 							match = True
		 				else:
							if predicate == True:
								match = True
	 					#if theres no match finish(its an OR we only need 1 match to return True)
	 					if match == True:
	 						return True

	 		return False
	 	else:
	 		raise Exception("Or statements disabled without :adl tag")


	def formatPreconditions(self, State, preconditions):
		""" Format the given preconditions to call to the unify algorithm

		Keyword arguments:
		State -- Pyhop State object
		preconditions -- Token stream with the preconditions list
		"""
		if preconditions[0] == "and" or preconditions[0] == "or":
			for i in range(1, len(preconditions)):
				self.formatPreconditions(State, preconditions[i])
		elif preconditions[0] == "not":
			self.formatPreconditions(State, preconditions[1:][0])
 		else:
 			self.unify(State, preconditions)


 	def unify(self, State, precondition):
 		""" Unifies the unknown variables of a given precondition with the
 		information stored in the state

		Keyword arguments:
		State -- Pyhop State object
		preconditions -- Token list representing a logic precondition
		"""
 		predicates = getattr(State, precondition[0].upper(), "Predicate_not_defined")
 		if predicates != "Predicate_not_defined":	
 			aux = []
			matches = 0
			for i in range(1, len(precondition)):
				if precondition[i] in self.linkedParams:
					matches += 1
					aux.append(self.linkedParams[precondition[i]])
				else:
					aux.append("__var__")
			for i in range(len(predicates)):
				auxMatches = 0
				for j in range(len(aux)):
					if aux[j] != "__var__":
						if self.linkedParams[precondition[j+1]] == predicates[i][j]:
							auxMatches = auxMatches + 1
				if auxMatches == matches:
					for j in range(len(aux)):
						if aux[j] == "__var__":
							if precondition[j+1] in self.unifiedParams:
								varAux = self.unifiedParams[precondition[j+1]]	
								if not predicates[i][j] in varAux:
									varAux.append(predicates[i][j])
									self.unifiedParams.update({precondition[j+1]:varAux})
							else:
								varAux = []
								varAux.append(predicates[i][j])
								self.unifiedParams.update({precondition[j+1]: varAux})
		else:
			raise Exception(precondition[0].upper() + " not defined")

		
	def calculateVars(self, pendingVars, state, preconditions):
		""" Generates all the posible combination of variables computed by the
		unification algorithm. Then checks it viability.

		Keyword arguments:
		pendingVars -- List of unchecked variables 
		state -- Pyhop State object
		preconditions -- Token stream with the preconditions list
		"""
		if len(pendingVars) == 0:
			return self.checkPreconditions(state, preconditions)
		else:
			values = self.unifiedParams[pendingVars[0]]
			for i in range(len(values)):
				self.linkedParams.update({pendingVars[0]:values[i]})
				if self.calculateVars(pendingVars[1:], state, preconditions) == True:
					return True

		return False

	def cleanLinkedParams(self):
		""" Resets the linkedParams dictionary

		"""
		for k in self.unifiedParams.keys():
			if k in self.linkedParams:
				del self.linkedParams[k]


	def formatTasks(self, tasks):
		""" Returns a aceptable task list usable by Pyhop

		Keyword arguments:
		tasks -- Token stream with the task to be formated
		"""
		formatedTasks = []
		for i in range(len(tasks)):
			aux = []
			aux.append(tasks[i][0].upper())
			for j in range(1, len(tasks[i])):
				aux.append(self.linkedParams[tasks[i][j]])
			formatedTasks.append(aux)
		
		return formatedTasks