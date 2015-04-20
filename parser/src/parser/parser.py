#!/usr/bin/python

from tokenGenerator import *
from domain import *


def makePrimitives(tokens):
	name = tokens[0]
	for i in range(len(tokens)):
		if tokens[i] == ":parameters":
			parameters = tokens[i+1]
		if tokens[i] == ":precondition":
			condition = tokens[i+1]
		if tokens[i] == ":effect":
			effect = tokens[i+1]
	if parameters == []:
		raise Exception("Parameters not defined")
	if condition == []:
		raise Exception("Conditions not defined")
	if effect == []:
		raise Exception("Effects not defined")

	"""
	print("\n" + "#################################################")
	print(parameters)
	print("#################################################")

	print("\n" + "#################################################")
	print(condition)
	print("#################################################")

	print("\n" + "#################################################")
	print(effect)
	print("#################################################")
	"""

def evaluateTokenList(tokens):
		if tokens ==  "define":
			global domain
			domain = Domain()

		elif tokens[0] == "domain":
			domain.setName(tokens[1])
		
		elif tokens[0] == ":requirements":
			domain.setRequirements(tokens[1:])

		elif tokens[0] == ":types":
			if domain.typing == True:
				domain.setTypes(tokens[1:])
			else:
				raise Exception("Trying to define types when :typing is not set in requirements")

		elif tokens[0] == ":predicates":
			domain.setPredicates(tokens[1:])

		elif tokens[0] == ":action":
			print(tokens)
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
		

		print(domain.name)
		print(domain.strips)
		print(domain.equility)
		print(domain.typing)
		print(domain.adl)
		print(domain.types)
		domain.printState()


if __name__ == '__main__':
	#parse("pddl/d-zenotravel-V00.pddl")
	#parse("pddl/Primitivas-ZenoTravel.pddl")
	#parse("pddl/problema-zeno-v01.pddl")
	parse("src/PFG/parser/src/parser/pddl/dominio1.pddl")

