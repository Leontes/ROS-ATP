#!/usr/bin/python

class Method(object):
	"""docstring for Method"""
	def __init__(self, name, domain, parameters, cases):
		self.__name__ = name.upper()
		self.domain = domain
		self.setParameters(parameters)
		self.cases = self.setCases(cases)


	def setParameters(self, parameters):
		self.params = []
		for i in range(len(parameters)):
			if parameters[i] == "-":
				if parameters[i+1].upper() in self.domain.types:
					self.params.append([parameters[i-1], parameters[i+1].upper()])
				else:
					raise Exception ("Type " + parameters[i+1].upper() + " not defined")


	def setCases(self, cases):
		print (cases)
		