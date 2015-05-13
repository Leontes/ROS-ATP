#!/usr/bin/python

from parserlib.tokenGenerator import *
from parserlib.domain import *
from parserlib.symbols import *
from parserlib.primitive import *
from parserlib.method import *


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
	
	if precondition == []:
		raise Exception("Conditions not defined")
	if effect == []:
		raise Exception("Effects not defined")

	problemDomain.setTasks(Primitive(name, problemDomain, parameters, precondition, effect))


def makeMethods(tokens):
	name = tokens[0]

	cases = []

	for i in range(len(tokens)):
		if tokens[i] == ":parameters":
			parameters = tokens[i+1]
		if tokens[i][0] == ":method":
			cases.append(tokens[i])

	if parameters == []:
		raise Exception("Parameters not defined")
	if cases == []:
		raise Exception("Parameters not defined")

	problemDomain.setMethods(name.upper(), Method(name, problemDomain, parameters, cases))


def makeGoals(tokens):
	aux = []
	tAux = ()
	for i in range(len (tokens)):
		tokens[i][0] = tokens[i][0].upper()
		tAux = tuple(tokens[i])
		aux.append(tAux)
	return aux
	

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

	elif tokens[0] == ":constants":
		problemDomain.setObjects(tokens[1:])

	elif tokens[0] == ":predicates":
		problemDomain.setPredicates(tokens[1:])

	elif tokens[0] == ":action":
		makePrimitives(tokens[1:])

	elif tokens[0] == ":task":
		makeMethods(tokens[1:])

	elif tokens[0] == "problem":
		problemDomain.setProblemName(tokens[1])

	elif tokens[0] == ":objects":
		problemDomain.setObjects(tokens[1:])

	elif tokens[0] == ":INIT":
		problemDomain.initState(tokens[1:])

	elif tokens[0] == ":goal":
		problemDomain.setGoals(makeGoals(tokens[1:]))

	else: 
		raise Exception("Unexpected token " + str(tokens[0]) + " received")


def parse(domainFilename, problemFilename):
	with open(domainFilename) as domainPddlFile:
		tkGen = tokenGenerator(domainPddlFile)
		tokens = tkGen.readAll()

		while tokens != eof_object:
			for i in range(len(tokens)):
				evaluateTokenList(tokens[i])	
			tokens = tkGen.readAll() 

	with open(problemFilename) as problemPddlFile:
		tkGen = tokenGenerator(problemPddlFile)
		tokens = tkGen.readAll()

		while tokens != eof_object:
			for i in range(len(tokens)):
				evaluateTokenList(tokens[i])
			tokens = tkGen.readAll()


	return problemDomain
