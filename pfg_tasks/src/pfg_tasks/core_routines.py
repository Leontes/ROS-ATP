from pi_trees_ros.pi_trees_ros import *
from std_msgs.msg import Float32
from pfg_tasks.core_tasks import *
from pfg_tasks import global_vars

def addCoreRoutines():
	batteryRoutine()


def batteryRoutine():
	# The "stay healthy" rutine
	stayHealthyTask = Selector("stayHealthy")

	with stayHealthyTask:
		# Add the check battery condition (uses MonitorTask)
		checkBatteryTask = MonitorTask("checkBattery", "battery_level", Float32, check_battery)



		# Add the recharge task (uses ServiceTask)
		chargeRobotTask = ServiceTask("chargeRobot", "battery_simulator/set_battery_level", SetBatteryLevel, 100, result_cb=recharge_cb)

		# Add the movement routine to the dock
		coords = global_vars.black_board.getCoords('dock')
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = 'map'
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose = coords
		
		moveToDockTask = SimpleActionTask("MoveToDock", "move_base", MoveBaseAction, goal, reset_after=True,  feedback_cb=update_robot_position)
		checkLocationTask = CheckLocation("dock")

		NavigationTask = Selector("Nav", [checkLocationTask, moveToDockTask] )

		# Build the recharge sequence using inline syntax
		rechargeTask = Sequence("recharge", [NavigationTask, chargeRobotTask])



		# Add the check battery and recharge tasks to the stay healthy selector
		stayHealthyTask.add_child(checkBatteryTask)
		stayHealthyTask.add_child(rechargeTask)

	global_vars.black_board.setRoutine(stayHealthyTask)