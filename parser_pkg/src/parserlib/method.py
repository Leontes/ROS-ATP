#!/usr/bin/python

class Method(object):
	"""docstring for Method"""
	def __init__(self, name, domain, parameters, cases):
		self.__name__ = name.upper()
		self.domain = domain
		self.setParameters(parameters)
		self.cases = self.setCases(cases)


	def  __call__(self, State, *args):
		if(self.checkArgs(State, args) == True):
			return self.execCases(State, args) 
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
		return True


	def setCases(self, cases):
		auxDict = {}
		for i in range(len(cases)):
			auxDict.update({cases[i][1]: cases[i][2:]})

		return auxDict


	def execCases(self, State, args):
		for i in self.cases:

			prec = self.cases[i][1]
			tasks = self.cases[i][3]

			#print prec
			#print tasks
			
			if self.checkPreconditions(State, prec) == True:
				pass

		return False


	def checkPreconditions(self, State, preconditions):
		self.unify(State, preconditions)
		if preconditions[0] == "and":
			return self.checkPreconditionsAnd(State, preconditions[1:])
		elif preconditions[0] == "or":
			return self.checkPreconditionsOr(State, preconditions[1:])
		prec = getattr(State, preconditions[0].upper(), False)
		if prec == False:
			raise Exception("Token " + str(preconditions[0]).upper() + " not defined")
		else:
			aux = []
			for i in range(1, len(preconditions)):
				#Filling the auxiliar list...
				aux.append(self.linkedParams[preconditions[i]])
			match = False
			#Check with the state
			for j in range(len(prec)):
				if aux == prec[j]:
					match = True

			return match




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
 					#Check with the state
 					for j in range(len(predicate)):
 						if aux == predicate[j]:
 							match = True
 					#if theres no match finish(its an AND we only need 1 missmatch to return False)
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
	 					#Check with the state
	 					for j in range(len(predicate)):
	 						if aux == predicate[j]:
	 							match = True
	 					#if theres no match finish(its an OR we only need 1 match to return True)
	 					if match == True:
	 						return True

	 		return False
	 	else:
	 		raise Exception("Or statements disabled without :adl tag")


	def unify(self, State, preconditions):
 		self.selectVars(State, preconditions)


 	def selectVars(self, State, preconditions):
 		print preconditions
		for i in range(len(preconditions)):
			aux = []
			matches = 0
			for j in range(1, len(preconditions[i])):
				print preconditions[i][j]
				if preconditions[i][j] in self.linkedParams:
					matches += 1
					aux.append(preconditions[i][j])
				else:
					aux.append("__var__")
			print preconditions[i][0]
			predicates = getattr(State, preconditions[i][0].upper(), False)
			print predicates
			#print preconditions[i][0]
			if predicates != False:	
				for j in range(len(predicates)):	
					auxMatchs = 0
					for k in range(len(predicates[j])):
						if aux[k] != "__var__":
							if aux[k] == predicates[j][k]:
								auxMatchs += 1
					print auxMatchs
					print matches
					if auxMatchs == matches:
						#Guardamos las variables en su sitio
						#print predicates[j]
						pass

		