from tokenGenerator import *
from os import path

def parse(filename):
	with open(filename) as pddlFile:
		#print(pddlFile.read())
		tkGen = tokenGenerator(pddlFile)
		tokens = tkGen.readAll()
		
		for i in range(len(tokens)):
			print(tokens[i])

if __name__ == '__main__':
	parse("pddl/d-zenotravel-V00.pddl")
	parse("pddl/Primitivas-ZenoTravel.pddl")
	parse("pddl/problema-zeno-v01.pddl")

