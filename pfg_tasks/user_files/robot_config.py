from pfg_tasks import global_vars

from pfg_tasks.core_tasks import *
from pfg_tasks.core_routines import *

from user_files.user_tasks import *
from user_files.user_routines import *


def getConfig():

	global_vars.black_board.setMovementTask('moveRobot')

	global_vars.black_board.setTask('pickUp',pickUpTask(name = 'pickUpTask', timer=60))
	global_vars.black_board.setTask('putDown',putDownTask(name = 'putDownTask', timer=60))
	global_vars.black_board.setTask('load',putDownTask(name = 'loadTask', timer=40))
	global_vars.black_board.setTask('unLoad',pickUpTask(name = 'unLoadTask', timer=40))
	global_vars.black_board.setTask('useWorkstation',sleepTask(name = 'useWorkstationTask', timer = 5))



	global_vars.black_board.setCoords("robot", 0, 0)
	global_vars.black_board.setCoords("storehouse", 0, 0)
	global_vars.black_board.setCoords("workstation1", 2, 1)
	global_vars.black_board.setCoords("dock", 0.5, 0.5)

	global_vars.black_board.setRobotOrigin("storehouse")

	addCoreRoutines()