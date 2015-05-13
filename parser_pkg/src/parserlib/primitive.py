#!/usr/bin/python

class Primitive (object):
	"""docstring for primitive """
	def __init__(self, name, domain, parameters, preconditions, effects):
		self.__name__ = name.upper()
		self.domain = domain
		self.setParameters(parameters)
		self.preconditions = preconditions
		if effects[0] == "and": 
			self.effects = effects[1:]
		else:
			self.effects = [effects]


	def  __call__(self, State, *args):
		if(self.checkArgs(State, args) == True):
			if(self.checkPreconditions(State, self.preconditions) == True):
				State1 = self.execPrimitiveEffects(State, args)
				return State1
		return False

	def setParameters(self, parameters):
		self.params = []
		for i in range(len(parameters)):
			if parameters[i] == "-":
				if parameters[i+1].upper() in self.domain.types:
					self.params.append([parameters[i-1], parameters[i+1].upper()])
				else:
					raise Exception ("Type " + parameters[i+1].upper() + " not defined")


	def checkArgs(self, state, *args):
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
		if preconditions[0] == "and":
			return self.checkPreconditionsAnd(State, preconditions[1:])
		elif preconditions[0] == "or":
			return self.checkPreconditionsOr(State, preconditions[1:])
		elif preconditions[0] == "not":
			return self.checkPreconditionsNot(State, preconditions[1:])
		prec = getattr(State, preconditions[0].upper(), False)
		if prec == False:
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
		preconditions = preconditionAuxList[0]
		if preconditions[0] == "and":
			return not self.checkPreconditionsAnd(State, preconditions[1:])
		elif preconditions[0] == "or":
			return not self.checkPreconditionsOr(State, preconditions[1:])
		elif preconditions[0] == "not":
			return not self.checkPreconditionsNot(State, preconditions[1:])
		prec = getattr(State, preconditions[0].upper(), False)
		if prec == False:
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
 				predicate = getattr(State, precondition[0].upper(), False)
 				#Lexic control
 				if predicate == False:
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
	 				predicate = getattr(State, precondition[0].upper(), False)
	 				#Lexic control
	 				if predicate == False:
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
		effect = []
		newState = State
		for i in range(len(self.effects)):
			effect = self.effects[i]
			if effect[0] == "not":
				effect = effect[1]
				aux = []
	 			for j in range(1, len(effect)):
	 				aux.append(self.linkedParams[effect[j]])

	 			auxEffect = getattr(newState, effect[0].upper(), False)
	 			if len(aux) != 0:
		 			eraseList= []
					if auxEffect != False:
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

	 			auxEffect = getattr(newState, effect[0].upper(), False)
	 			if len(aux) != 0:
					if auxEffect != False:
						if auxEffect != "__NON_DEFINED__":
							auxEffect.append(aux)
							setattr(newState, effect[0].upper(), auxEffect)
						else:
							auxEffect = aux
							setattr(newState, effect[0].upper(), auxEffect)
					else:
						raise Exception(effect[0].upper() + " predicate not defined")
				else:
					setattr(newState, effect[0].upper(), True)

		return newState
		
		