#!/usr/bin/env python

"""
Factory World domain definition for Pyhop 1.1.
Author: Jose Angel Segura Muros  <shaljas@correo.ugr.es>, February 17, 2015
"""
from pyhop import hop


"""The blocks-world operators use five state variables:
- types[X] = piece X type, in the example code it can be type1 or type2
- position[X] = X can be a piece or a robot, if it refers to a piece the variable shows that the piece is in a storehouse/workstation/robot. 
				If X refers to a robot it shows that the robot is near a storehouse/workstation.
- ocupied[X] = it indicates if a robot/workstation X is ocupied, the variable shows name of the block being held, or False if the hand is empty.
- stationAcepts[X] = it indicates what type of pieces acepts the workstation X to begin to work
- stationProduces[X] = it indicates what type of pieces creates the workstation X when it ends the work
"""


"""
Move the robot to the desired position, if the robot already is in that position the operator fails
"""
def moveRobot(state,location):
	if state.position['robot'] != location and location != 'robot':
		state.position['robot'] = location
		return state
	else:
		return False


"""
Operator that picks a piece if it is in the storehouse, in another case it fails
"""
def pickUp(state, piece):
	if state.position['robot'] == state.position[piece] and state.position[piece] == 'storehouse' and state.ocupied['robot'] == False:
		state.ocupied['robot'] = piece
		state.position[piece] = 'robot'
		return state
	else:
		return False


"""
Operator that puts a piece in the storehouse
"""
def putDown(state, piece):
	if state.position['robot'] == 'storehouse' and state.ocupied['robot'] == piece:
		state.ocupied['robot'] = False
		state.position[piece] = 'storehouse'
		return state
	else:
		return False


"""
Operator that loads a workstation with a piece, if the workstation is busy or not acepts the piece it fails
"""
def load(state, workstation, piece):
	if state.position['robot'] == workstation and state.ocupied[workstation] == False and state.ocupied['robot'] == piece and state.types[piece] == state.stationAcepts[workstation]:
		state.ocupied[workstation] = piece
		state.ocupied['robot'] = False
		state.position[piece] = workstation
		return state
	else:
		return False


"""
Operator that unloads a workstation, if the robot is busy it fails
"""
def unLoad(state, workstation, piece):
	if state.position['robot'] == workstation and state.ocupied[workstation] == piece and state.ocupied['robot'] == False:
		state.ocupied[workstation] = False
		state.ocupied['robot'] = piece
		state.position[piece] = 'robot'
		return state
	else:
		return False


"""
Operator that uses a workstation to transform a piece
"""
def useWorkstation(state, workstation):
	if state.position['robot'] == workstation and state.ocupied['robot'] == False and state.ocupied[workstation] != False:
		state.types[state.ocupied[workstation]] = state.stationProduces[workstation]
		return state
	else:
		return False



hop.declare_operators(moveRobot, pickUp, putDown, load, unLoad, useWorkstation)
