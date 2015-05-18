#!/usr/bin/python

"""
primitive.py - Version 1.0 2015-05-15

Executable class thats implements a generic pyhop task

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

class Primitive (object):
	""" class Primitive

	"""


	def __init__(self, name, domain, parameters, preconditions, effects):
		""" Construct a object of the class Primitive

		Keyword arguments:
		name -- String, name of the task
		domain -- Domain object asociated to the task
		parameters -- Token stream with the parameters of the task
		preconditions -- Token stream with the preconditions of the task
		effects -- Token stream with the effects of the task
		"""
		self.__name__ = name.upper()
		self.domain = domain
		self.setParameters(parameters)
		self.preconditions = preconditions
		if effects[0] == "and": 
			self.effects = effects[1:]
		else:
			self.effects = [effects]


	def  __call__(self, State, *args):
		""" Executes the task object

		Keyword arguments:
		State -- Pyhop State object 
		args -- Arguments of the execution
		"""
		if(self.checkArgs(State, args) == True):
			if(self.checkPreconditions(State, self.preconditions) == True):
				State1 = self.execPrimitiveEffects(State, args)
				return State1
		return False


	def setParameters(self, parameters):
		""" Set the parameters of the task

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
		""" Check the arguments used to call the task

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
				#Filling the auxiliar list...
				aux.append(self.linkedParams[preconditions[i]])
			if len(aux) != 0:
				match = False
				#Check with the state
				for j in range(len(prec)):
					if aux == prec[j]:
						match = True

				return match
			else:
				if prec == "__NON_DEFINED__" or prec == False:
					return False
				else:
					return True


	def checkPreconditionsNot(self, State, preconditionAuxList):
		""" Checks the given NOT precondition with que state

		Keyword arguments:
		State -- Pyhop State object
		preconditions -- Token stream with the preconditions list
		"""
		preconditions = preconditionAuxList[0]
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
				#Filling the auxiliar list...
				aux.append(self.linkedParams[preconditions[i]])
			if len(aux) != 0:
				match = False
				#Check with the state
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


	def execPrimitiveEffects(self, State, *args):
		""" Modifies a given state with the call arguments and the programmed effects
		of the task

		Keyword arguments:
		state -- Pyhop State object
		args -- Arguments of the execution
		"""
		effect = []
		newState = State
		for i in range(len(self.effects)):
			effect = self.effects[i]
			if effect[0] == "not":
				effect = effect[1]
				aux = []
	 			for j in range(1, len(effect)):
	 				aux.append(self.linkedParams[effect[j]])

	 			auxEffect = getattr(newState, effect[0].upper(), "Predicate_not_defined")
	 			if len(aux) != 0:
		 			eraseList= []
					if auxEffect != "Predicate_not_defined":
						if auxEffect != "__NON_DEFINED__":
							for k in range(len(auxEffect)):
								if aux != auxEffect[k]:
									eraseList.append(auxEffect[k])
							setattr(newState, effect[0].upper(), eraseList)
						else:
							auxEffect = aux
							setattr(newState, effect[0].upper(), auxEffect)
					else:
						raise Exception(effect[0].upper() + " predicate not defined")
				else:
					setattr(newState, effect[0].upper(), False)

			else:
				aux = []
	 			for j in range(1, len(effect)):
	 				aux.append(self.linkedParams[effect[j]])

	 			auxEffect = getattr(newState, effect[0].upper(), "Predicate_not_defined")
	 			if len(aux) != 0:
					if auxEffect != "Predicate_not_defined":
						if auxEffect != "__NON_DEFINED__":
							auxEffect.append(aux)
							setattr(newState, effect[0].upper(), auxEffect)
						else:
							auxEffect = []
							auxEffect.append(aux)
							setattr(newState, effect[0].upper(), auxEffect)
					else:
						raise Exception(effect[0].upper() + " predicate not defined")
				else:
					setattr(newState, effect[0].upper(), True)

		return newState
		
		