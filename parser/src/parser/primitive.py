#!/usr/bin/python

class primitive (object):
	"""docstring for primitive """
	def __init__(self, name, parameters, duration, preconditions, effects):
		self.__name__ = name
		self.params = parameters
		self.preconditions = preconditions
		self.effects = effects


	def  __call__(self, State, *args):
		if(checkArgs(args) == True):
			if(checkConditions(State) == True)
				State1 = execPrimitiveEffects(State1)
				return State1
		return False


	def checkArgs(self, *args):
		pass


	def checkConditions(self, State):
		pass


	def execPrimitiveEffects(self, State):
		pass
		
		