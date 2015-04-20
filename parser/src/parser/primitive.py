class primitive (object):
	"""docstring for primitive """
	def __init__(self, name, parameters, duration, conditions, effects):
		self.__name__ = name
		self.params = parameters
		self.duration = duration
		self.conditions = conditions
		self.effects = effects

	def  __call__(self, State, *args):
		if(checkArgs(args) == True):
			if(checkConditions(State) == True)
				State1 = execDurationEffects(State)
				State1 = execPrimitiveEffects(State1)
				return State1
		return False

	def checkArgs(self, *args):
		pass


	def checkConditions(self, State):
		pass


	def execDurationEffects(self, State):
		pass


	def execPrimitiveEffects(self, State):
		pass
		
		