#!/usr/bin/env python


from __future__ import print_function

import rospy

from pyhop_pkg.pyhop import *
from pyhop_pkg.factory_methods import *
from pyhop_pkg.factory_operators import *
from pi_trees_lib.pi_trees_lib import *
from pfg_tasks.task_setup import * 
from pfg_tasks import global_vars
from user_files.robot_config import *
from pfg_tasks.core_tasks import *


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
		
		rospy.init_node("planner_node", anonymous=False)

		rospy.on_shutdown(self.shutdown)

		setup_task_environment(self)

		global_vars.init()

		getConfig()

		Tree = makeTree(plan)

		print_tree(Tree)


		while not rospy.is_shutdown():
			Tree.run()
			rospy.sleep(0.1)


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