from tasks import global_vars

from tasks.core_tasks import *
from tasks.core_routines import *

from user_files.user_tasks import *
from user_files.user_routines import *


def getConfig():
	"""Sets the black board configuration 

	"""

	#Indicates the movement action defined in the planification domain
	global_vars.black_board.setMovementTask('GO', 2)

	#Indicates the relation between a action of the plan and a executable task
	global_vars.black_board.setTask('PICK_UP',
		spinRightTask(name = 'pickUpTask', timer=60))
	global_vars.black_board.setTask('PUT_DOWN',
		spinLeftTask(name = 'putDownTask', timer=60))
	global_vars.black_board.setTask('LOAD',
		spinLeftTask(name = 'loadTask', timer=40))
	global_vars.black_board.setTask('UNLOAD',
		spinRightTask(name = 'unLoadTask', timer=40))
	global_vars.black_board.setTask('TRANSFORM',
		sleepTask(name = 'useWorkstationTask', timer = 50))
	#global_vars.black_board.setTask('actionName', 
		#task(param1 = value1, param2 = value2, ...))

	#Indicates the [x,y] coordinates in the simulator of any entity 
	#in the planification domain
	global_vars.black_board.setCoords("storehouse", 0, 0)
	global_vars.black_board.setCoords("workshop1", 2.1, 2.8)
	global_vars.black_board.setCoords("workshop2", -0.2, -1.4)
	global_vars.black_board.setCoords("dock", 2, -1.6)
	#global_vars.black_board.setCoords("entity", xValue, yValue)

	#Sets the initial position of the robot, must be the same
	#as in the initial state of the planner
	global_vars.black_board.setRobotOrigin("storehouse")


	#Adds the routines 
	addCoreRoutines()
	addUserRoutines()