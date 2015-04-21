#!/usr/bin/python

from pyhop import hop

class Domain(object):
	"""docstring for Domain"""
	def __init__(self):
		self.state = hop.State("Initial State")


	def setName(self, name):
		self.name = name


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
		self.types = types

	def setPredicates(self, predicates):
		self.objTypes = {}
		for i in range(len (predicates)):
			objAux = []
			for j in range(len(predicates[i])):
				if predicates[i][j] == "-":
					if not (predicates[i][j+1] in self.types):
						raise Exception(predicates[i][j+1] + " type not defined")
					else:
						objAux.append(predicates[i][j+1])
			self.objTypes.update({predicates[i][0]:objAux})
			setattr(self.state, predicates[i][0], False)


	def printState(self):
		hop.print_state(self.state)