#!/usr/bin/env python

"""
Factory methods definition for Pyhop 1.1.
Author: Jose Angel Segura Muros  <shaljas@correo.ugr.es>, February 17, 2015
"""

import pyhop_pkg.pyhop


"""
Here are some helper functions that are used in the methods' preconditions.
"""

def is_done(piece,state,goal):
    if piece in goal.types:
        if goal.types[piece] != state.types[piece]:
             return False
        if goal.position[piece] != state.position[piece]: 
            return False
    return True


def status(piece,state,goal):
    if is_done(piece,state,goal):
        return 'done'
    elif state.types[piece] == goal.types[piece] and state.position[piece] != goal.position[piece]:
        return 'move-to-position'
    elif state.types[piece] != goal.types[piece] and state.position[piece] != 'storehouse' and state.position[piece] != 'robot' and state.stationProduces[state.position[piece]] != state.types[piece]:
        return 'ready-to-be-transformated'
    elif state.types[piece] != goal.types[piece] and (state.position[piece] == 'storehouse' or state.position[piece] == 'robot'):
        if workstationAvailable(state, piece) != False:
            return 'move-to-workstation'
        else:
            return 'waiting'
    elif state.types[piece] != goal.types[piece] and state.position[piece] != 'storehouse' and state.position[piece] != 'robot' and state.stationProduces[state.position[piece]] == state.types[piece]:
        if workstationAvailable(state, piece) != False:
            return 'move-to-workstation'
        else:
            return 'waiting'
    else:
        return 'waiting'


def all_pieces(state):
    return state.types.keys()


def robot_in_position(state,goal):
	return state.position['robot'] == goal.position['robot']


def workstationAvailable(state, piece):
    for workstation in state.stationAcepts.keys():
        if state.stationAcepts[workstation] == state.types[piece]:
            if(state.ocupied[workstation] == False):
                return workstation

    return False

"""
Factory methods
"""

def get(state,piece):
    """
    Generate either a pickUp or an unLoad subtask for piece.
    """
    if state.ocupied['robot'] == False:
        if state.position[piece] == 'storehouse':
                return [('pickUp',piece)]
        else:
                return [('unLoad',state.position[piece],piece)]
    else:
        return False

pyhop_pkg.pyhop.declare_methods('get',get)


def put(state,piece):
    """
    Generate either a putDown or a load subtask for piece.
    """
    if state.ocupied['robot'] != False:
        if state.position['robot'] == 'storehouse':
                return [('putDown',piece)]
        else:
                return [('load',state.position['robot'],piece)]
    else:
        return False

pyhop_pkg.pyhop.declare_methods('put',put)


def movePiece(state,piece,dest):
    """
    Generate subtasks to get b1 and put it at dest.
    """
    return [('get', piece), ('navigateRobot',dest), ('put', piece)]

pyhop_pkg.pyhop.declare_methods('movePiece',movePiece)


def navigateRobot(state, dest):
    """
    If the robot is already at dest, doesn't anything
    """
    if state.position['robot'] != dest:
        return [('moveRobot', dest)]
    else:
        return []

pyhop_pkg.pyhop.declare_methods('navigateRobot', navigateRobot)


def work(state, goal):
    """This methods implements the following algorithm:
		- If a piece can be transformated to another type, the robot uses the workstation
		- If a piece has been already transformed to another type, it the robot puts them in the storehouse or in another workstation
		- If a piece needs to be transformed the robot puts them in an apropiated workstation
        - If there's nothing to do with any piece, moves the robot to is final position 
	"""
    for piece in all_pieces(state):
        piece_status = status(piece, state, goal)
        if piece_status == 'ready-to-be-transformated':
            return [('navigateRobot', state.position[piece]), ('useWorkstation', state.position[piece]), ('work',goal)]
        if piece_status == 'move-to-position':
            return [('navigateRobot', state.position[piece]), ('movePiece',piece, goal.position[piece]), ('work',goal)]
        if piece_status == 'move-to-workstation':
            station = workstationAvailable(state, piece)
            if station != False:
                return [('navigateRobot', state.position[piece]), ('movePiece',piece, station), ('work',goal)]
            else:
                return [('navigateRobot', state.position[piece]), ('movePiece',piece, 'storehouse'), ('work',goal)]
    if robot_in_position(state,goal) == False:
        return [('navigateRobot', goal.position['robot']), ('work',goal)]
    
    return []
pyhop_pkg.pyhop.declare_methods('work',work)