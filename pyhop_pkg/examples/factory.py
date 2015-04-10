#!/usr/bin/env python

"""
Factory test data for Pyhop 1.1.
Author: Jose Angel Segura Muros  <shaljas@correo.ugr.es>, February 17, 2015
"""
from __future__ import print_function
from pyhop import *

import factory_operators
print('')
print_operators()

import factory_methods
print('')
print_methods()


print("""
****************************************
First, we define the first state for the problem
****************************************
""")

print("- Define state: piece1 of type type1, piece1 on storehouse, robot on storehouse, workstation1 transforms type1 pieces to type2")

state = State('state')
state.types={'piece1':'type1'}
state.position={'piece1':'storehouse', 'robot':'storehouse'}
state.ocupied={'robot':False, 'workstation1':False}
state.stationAcepts={'workstation1':'type1'}
state.stationProduces={'workstation1':'type2'}

print_state(state)
print('')


print("""
****************************************
Next, we define the test goal for the problem
****************************************
""")

print("- Define goal: Make a type2 piece and put it in the storehouse")

goal = Goal('goal')
goal.types={'piece1':'type2'}
goal.position={'piece1':'storehouse', 'robot':'storehouse'}
goal.ocupied={'robot':False, 'workstation1':False}
goal.stationAcepts={'workstation1':'type1'}
goal.stationProduces={'workstation1':'type2'}

print_goal(goal)
print('')


print("""
****************************************
And at last, we run the planner
****************************************
""")

pyhop(state,[('work', goal)], verbose=1)


print("""
****************************************
Now... let's try a more ambitious problem
****************************************
""")


print("- Define new state: Now with 2 pieces, one of them in the workstation")

state2 = State('state')
state2.types={'piece1':'type1', 'piece2':'type1'}
state2.position={'piece1':'storehouse', 'piece2':'workstation1', 'robot':'storehouse'}
state2.ocupied={'robot':False, 'workstation1':'piece2'}
state2.stationAcepts={'workstation1':'type1'}
state2.stationProduces={'workstation1':'type2'}

print_state(state2)
print('')


print("- Define goal: Make two type2 pieces and puts them it in the storehouse")

goal2 = Goal('goal')
goal2.types={'piece1':'type2', 'piece2':'type2'}
goal2.position={'piece1':'storehouse', 'piece2':'storehouse', 'robot':'storehouse'}
goal2.ocupied={'robot':False, 'workstation1':False}
goal2.stationAcepts={'workstation1':'type1'}
goal2.stationProduces={'workstation1':'type2'}

print_goal(goal2)
print('')


print("""
****************************************
And the result
****************************************
""")

pyhop(state2,[('work', goal2)], verbose=1)



print("""
****************************************
And finally a real problem
****************************************
""")


print("- Define new state: 2 pieces, 3 workstations: 2 acepts type1 pieces, and 1 a type2")

state3 = State('state')
state3.types={'piece1':'type1', 'piece2':'type1'}
state3.position={'piece1':'storehouse', 'piece2':'workstation1', 'robot':'storehouse'}
state3.ocupied={'robot':False, 'workstation1':'piece2', 'workstation2':False, 'workstation3':False}
state3.stationAcepts={'workstation1':'type1', 'workstation2':'type1', 'workstation3':'type2'}
state3.stationProduces={'workstation1':'type2', 'workstation2':'type2', 'workstation3':'type3'}

print_state(state3)
print('')


print("- Define goal: Make a type2 piece and a type3 piece, puts them it in the storehouse")

goal3 = Goal('goal')
goal3.types={'piece1':'type3', 'piece2':'type2'}
goal3.position={'piece1':'storehouse', 'piece2':'storehouse', 'robot':'storehouse'}
goal3.ocupied={'robot':False, 'workstation1':False, 'workstation2':False, 'workstation3':False }
goal3.stationAcepts={'workstation1':'type1', 'workstation2':'type1', 'workstation3':'type2'}
goal3.stationProduces={'workstation1':'type2', 'workstation2':'type2', 'workstation3':'type3'}

print_goal(goal3)
print('')


print("""
****************************************
And the final result
****************************************
""")

pyhop(state3,[('work', goal3)], verbose=1)