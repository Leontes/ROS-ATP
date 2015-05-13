#!/usr/bin/env python
from parserlib import parser
from pyhop import hop


if __name__ == '__main__':

	domain = parser.parse("src/PFG/parser_pkg/src/parserlib/pddl/dominio1.pddl", "src/PFG/parser_pkg/src/parserlib/pddl/problema1.pddl")
	
	print "\nINITIAL STATE: \n"
	domain.printState()
	print "\n"


	hop.declare_operators(*(domain.taskList))
	hop.print_operators(hop.get_operators())

	for k in domain.methodList.keys():
		hop.declare_methods(k, domain.methodList[k])
	hop.print_methods(hop.get_methods())	

	print "\n"
	plan = hop.plan(domain.state,domain.getGoals(),hop.get_operators(),hop.get_methods(),verbose=2)
	print plan 