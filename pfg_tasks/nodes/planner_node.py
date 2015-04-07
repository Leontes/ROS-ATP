#!/usr/bin/env python


from __future__ import print_function

import rospy

from pyhop_pkg.pyhop import *
import pfg_tasks.factory_methods
import pfg_tasks.factory_operators
from pi_trees_lib.pi_trees_lib import *
from pfg_tasks.black_board import *
from pfg_tasks.robotTasks import *
from pfg_tasks.task_setup import * 


class Work():
 	"""docstring for Work"""
 	def __init__(self):

		state = State('state')
		state.types={'piece1':'type1'}
		state.position={'piece1':'storehouse', 'robot':'storehouse'}
		state.ocupied={'robot':False, 'workstation1':False}
		state.stationAcepts={'workstation1':'type1'}
		state.stationProduces={'workstation1':'type2'}

		goal = Goal('goal')
		goal.types={'piece1':'type2'}
		goal.position={'piece1':'storehouse', 'robot':'storehouse'}
		goal.ocupied={'robot':False, 'workstation1':False}
		goal.stationAcepts={'workstation1':'type1'}
		goal.stationProduces={'workstation1':'type2'}


		plan = pyhop(state,[('work', goal)], verbose=0)

		print('** result =',plan,'\n')



		black_board.setWorld(state)

		black_board.setWorldOperators(operators)

		black_board.setCoords("robot", 0, 0)
		black_board.setCoords("workstation1", 2, 1)
		black_board.setCoords("storehouse", 0, 0)
		black_board.setCoords("dock", 0.5, 0.5)



		black_board.setMovementTask('moveRobot')

		black_board.setTask('pickUp',pickUpTask(name = 'pickUpTask', timer=60))
		black_board.setTask('putDown',putDownTask(name = 'putDownTask', timer=60))
		black_board.setTask('load',putDownTask(name = 'loadTask', timer=40))
		black_board.setTask('unLoad',pickUpTask(name = 'unLoadTask', timer=40))
		black_board.setTask('useWorkstation',sleepTask(name = 'useWorkstationTask', timer = 5))

		
		rospy.init_node("planner_node", anonymous=False)
		
		batteryRoutine(black_board)

		

		rospy.on_shutdown(self.shutdown)

		setup_task_environment(self)


		Tree = black_board.makeTree(plan)

		print_tree(Tree)


		#while not rospy.is_shutdown():
		#	Tree.run()
		#	rospy.sleep(0.1)


        
	def update_robot_position(self, msg):
		black_board.robot_position = msg.base_position.pose.position


	def shutdown(self):
		rospy.loginfo("Stopping the robot...")
		self.move_base.cancel_all_goals()

		self.cmd_vel_pub.publish(Twist())

		rospy.sleep(1)


if __name__ == '__main__':
    try:
        Work()
    except rospy.ROSInterruptException:
        rospy.loginfo("House clearning test finished.")