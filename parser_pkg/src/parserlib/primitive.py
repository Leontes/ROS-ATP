#!/usr/bin/python

class Primitive (object):
	"""docstring for primitive """
	def __init__(self, name, domain, parameters, preconditions, effects):
		self.__name__ = name.upper()
		self.domain = domain
		self.setParameters(parameters)
		self.preconditions = preconditions
		self.effects = effects


	def  __call__(self, State, *args):
		if(checkArgs(args) == True):
			if(checkPreconditions(State) == True):
				State1 = execPrimitiveEffects(State1)
				return State1
		return False


	def setParameters(self, parameters):
		self.params = {}
		for i in range(len(parameters)):
			if parameters[i] == "-":
				if parameters[i+1].upper() in self.domain.types:
					self.params.update({parameters[i-1]:parameters[i+1]})
				else:
					raise Exception ("Type " + parameters[i+1].upper() + " not defined")


	def checkArgs(self, state, *args):
		for arg in args:
			if state.objTypes[arg] != self.params[arg]:
				return False
		return True



	def checkPreconditions(self, State):
		pass


	def execPrimitiveEffects(self, State):
		pass
		
		