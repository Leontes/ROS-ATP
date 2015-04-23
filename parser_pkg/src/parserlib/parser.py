#!/usr/bin/python

from parserlib.tokenGenerator import *
from parserlib.domain import *
from parserlib.symbols import *
from parserlib.primitive import *

from pyhop import hop


taskList = []

defined = False

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
	global defined
	if tokens ==  "define":
		if not defined:
			global problemDomain
			problemDomain = Domain()
			defined = True

	elif tokens[0] == "domain":
		problemDomain.setName(tokens[1])

	elif tokens[0] == ":domain":
		problemDomain.checkDomainName(tokens[1])
	
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

	elif tokens[0] == "problem":
		problemDomain.setProblemName(tokens[1])

	elif tokens[0] == ":objects":
		problemDomain.setObjects(tokens[1:])

	elif tokens[0] == ":INIT":
		problemDomain.initState(tokens[1:])

	elif tokens[0] == ":goal":
		pass

	else: 
		raise Exception("Unexpected token " + tokens[0] + " received")


def parse(domainFilename, problemFilename):
	with open(domainFilename) as domainPddlFile:
		#print(pddlFile.read())
		tkGen = tokenGenerator(domainPddlFile)
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

	with open(problemFilename) as problemPddlFile:
		#print(pddlFile.read())
		tkGen = tokenGenerator(problemPddlFile)
		tokens = tkGen.readAll()
		#evaluateTokenList(tokens)

		while tokens != eof_object:
			for i in range(len(tokens)):
				evaluateTokenList(tokens[i])
			tokens = tkGen.readAll()

	print("Domain name: " + problemDomain.name)
	print("Problem name: " + problemDomain.problemName)
	print(":strips enabled: " + str(problemDomain.strips))
	print(":equility enabled: " + str(problemDomain.equility))
	print(":typing enabled: "+ str(problemDomain.typing))
	print(":adl enabled: " + str(problemDomain.adl))
	print("\nTypes defined: " + str(problemDomain.types))
	print("\nObject list: " + str(problemDomain.objList))
	print("\nInitial state: ")
	problemDomain.printState()
	print("\n")
	hop.declare_operators(*taskList)
	hop.print_operators(hop.get_operators())

	print(taskList[3](problemDomain.state, "B", "A", "M1", "R1", "Rob1"))

	


if __name__ == '__main__':

	parse("src/PFG/parser_pkg/src/parserlib/pddl/dominio1.pddl", "src/PFG/parser_pkg/src/parserlib/pddl/problema1.pddl")

