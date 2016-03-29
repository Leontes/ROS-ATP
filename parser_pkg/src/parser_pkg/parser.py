#!/usr/bin/python

"""
parser.py - Version 1.0 2015-05-14

Pddl parser for pyhop

Copyright (c) 2015 Jose Angel Segura Muros.  All rights reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

"""


from parser_pkg.tokenGenerator import *
from parser_pkg.domain import *
from parser_pkg.primitive import *
from parser_pkg.method import *


defined = False

def makePrimitives(tokens):
	"""Formats a token stream and creates a new primitive task

	Keyword arguments:
	tokens -- Collection of tokens
	"""
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

	problemDomain.setTasks(Primitive(name, problemDomain, parameters,
		precondition, effect))


def makeMethods(tokens):
	"""Formats a token stream and creates a new method task

	Keyword arguments:
	tokens -- Collection of tokens
	"""

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
		raise Exception("Cases not defined")

	problemDomain.setMethods(name.upper(), Method(name, problemDomain,
		parameters, cases))


def makeGoals(tokens):
	"""Formats a token stream and creates a new goal

	Keyword arguments:
	tokens -- Collection of tokens
	"""
	aux = []
	tAux = ()
	for i in range(len (tokens)):
		tokens[i][0] = tokens[i][0].upper()
		tAux = tuple(tokens[i])
		aux.append(tAux)
	return aux


def evaluateTokenList(tokens):
	"""Formats a token stream and call the propers function to deal with it

	Keyword arguments:
	tokens -- Collection of tokens
	"""
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
			raise Exception("Trying to define types when :typing is not" +
				"set in requirements")

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
	"""Opens the files, generates a token list and process it

	Keyword arguments:
	domainFilename -- String, name of the domain file
	problemFilename -- String, name of the problem file
	"""
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
