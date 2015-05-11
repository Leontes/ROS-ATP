#!/usr/bin/python

from pyhop import hop

class Domain(object):
	"""docstring for Domain"""
	def __init__(self):
		self.state = hop.State("Initial State")
		self.objList = {}


	def setName(self, name):
		self.name = name


	def checkDomainName(self, name):
		if self.name != name:
			raise Exception("Domain names not match in the domain and problem files")

	def setProblemName(self, problem):
		self.problemName = problem


	def setRequirements(self, requirements):
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
		self.types = []
		for tp in types:
			self.types.append(tp.upper())

	def setPredicates(self, predicates):
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
		self.goals  = goals


	def getGoals(self):
		return self.goals


	def initState(self, initList):
		for i in range(len(initList)):
			#statePredicate = getattr(self.state, initList[i][0].upper(), False)
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





	def printState(self):
		hop.print_state(self.state)