#!/usr/bin/env python


"""
domain_checker.py - Version 1.0 2015-04-15

Test a given domain and problem with Pyhop

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


from parser_pkg import parser
from pyhop import hop
import sys



if __name__ == '__main__':

	if len(sys.argv) < 3:
		print("Usage: rosrun pfg_utils domain_checker.py domain.pddl problem.pddl")
	else:
		domPddl = sys.argv[1]
		probPddl = sys.argv[2]
		print "\nParsing domain: " + domPddl
		print "Parsing problem: " + probPddl
		domain = parser.parse(domPddl, probPddl)

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
