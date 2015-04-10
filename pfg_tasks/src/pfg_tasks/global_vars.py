from pfg_tasks.black_board import *

def init():
	global black_board 
	black_board = BlackBoard()

	global low_battery_threshold 
	low_battery_threshold = 50.0