#!/usr/bin/python

from parserlib.tokenGenerator import *
from parserlib.domain import *
from parserlib.symbols import *
from parserlib.primitive import *

from pyhop import hop

taskList = []

def makePrimitives(tokens):
	name = tokens[0]
	for i in range(len(tokens)):
		if tokens[i] == ":parameters":
			parameters = tokens[i+1]
		if tokens[i] == ":precondition":
			precondition = tokens[i+1]
		if tokens[i] == ":effect":
			effect = tokens[i+1]
	if parameters == []:
		raise Exception("Parameters not defined")
	if precondition == []:
		raise Exception("Conditions not defined")
	if effect == []:
		raise Exception("Effects not defined")

	taskList.append(Primitive(name, problemDomain, parameters, precondition, effect))
	

def evaluateTokenList(tokens):
		if tokens ==  "define":
			global problemDomain
			problemDomain = Domain()

		elif tokens[0] == "domain":
			problemDomain.setName(tokens[1])
		
		elif tokens[0] == ":requirements":
			problemDomain.setRequirements(tokens[1:])

		elif tokens[0] == ":types":
			if problemDomain.typing == True:
				problemDomain.setTypes(tokens[1:])
			else:
				raise Exception("Trying to define types when :typing is not set in requirements")

		elif tokens[0] == ":predicates":
			problemDomain.setPredicates(tokens[1:])

		elif tokens[0] == ":action":
			#print(tokens)
			makePrimitives(tokens[1:])

		else: 
			raise Exception("Unexpected token " + tokens[0] + " received")


def parse(filename):
	with open(filename) as pddlFile:
		#print(pddlFile.read())
		tkGen = tokenGenerator(pddlFile)
		tokens = tkGen.readAll()
		#evaluateTokenList(tokens)

		while tokens != eof_object:
			for i in range(len(tokens)):
				#print(tokens[i])
				#print (i)

				#print("\n" + "---------------------------------------------")
				evaluateTokenList(tokens[i])
				#print("---------------------------------------------")
			tokens = tkGen.readAll() 
		

		print(problemDomain.name)
		print(problemDomain.strips)
		print(problemDomain.equility)
		print(problemDomain.typing)
		print(problemDomain.adl)
		print(problemDomain.types)
		problemDomain.printState()


		hop.declare_operators(*taskList)
		hop.print_operators(hop.get_operators())


if __name__ == '__main__':
	#parse("pddl/d-zenotravel-V00.pddl")
	#parse("pddl/Primitivas-ZenoTravel.pddl")
	#parse("pddl/problema-zeno-v01.pddl")
	parse("src/PFG/parser_pkg/src/parserlib/pddl/dominio1.pddl")

