from tokenGenerator import *
from os import path


def makePrimitives(tokens):
	name = tokens[0]
	for i in range(len(tokens)):
		if tokens[i] == ":parameters":
			parameters = tokens[i+1]
		if tokens[i] == ":duration":
			duration = tokens[i+1]
		if tokens[i] == ":condition":
			condition = tokens[i+1]
		if tokens[i] == ":effect":
			effect = tokens[i+1]
	if parameters == []:
		raise("Parameters not defined")
	if duration == []:
		raise("Duration not defined")
	if condition == []:
		raise("Conditions not defined")
	if effect == []:
		raise("Effects not defined")

	print("\n\n\n" + "#################################################")
	print(parameters)
	print("#################################################" + "\n\n\n")

	print("\n\n\n" + "#################################################")
	print(duration)
	print("#################################################" + "\n\n\n")

	print("\n\n\n" + "#################################################")
	print(condition)
	print("#################################################" + "\n\n\n")

	print("\n\n\n" + "#################################################")
	print(effect)
	print("#################################################" + "\n\n\n")


def evaluateTokenList(tokens):
		if tokens[0] ==  ":durative-action":
			print ("Aqui hacemos una primitiva")
			makePrimitives(tokens[1:])

		else: 
			raise("Unexpected token " + tokens[0] + " received")


def parse(filename):
	with open(filename) as pddlFile:
		#print(pddlFile.read())
		tkGen = tokenGenerator(pddlFile)
		tokens = tkGen.readAll()
		evaluateTokenList(tokens)
"""
		while tokens != eof_object:
			for i in range(len(tokens)):
				print(tokens[i])
				print (i)

			print("\n\n\n" + "#################################################")
			evaluateTokenList(tokens)
			print("#################################################" + "\n\n\n")
			tokens = tkGen.readAll() 
"""

if __name__ == '__main__':
	#parse("pddl/d-zenotravel-V00.pddl")
	parse("pddl/Primitivas-ZenoTravel.pddl")
	#parse("pddl/problema-zeno-v01.pddl")

