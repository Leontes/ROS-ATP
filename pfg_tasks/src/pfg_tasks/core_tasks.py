import rospy
from pi_trees_ros.pi_trees_ros import *
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Twist

from math import sqrt

from pfg_tasks import global_vars
from pfg_msgs.srv import *



class pickUpTask(Task):
	"""docstring for pickUpTask"""
	def __init__(self, name, timer, *args):
		super(pickUpTask, self).__init__(name, children = None, *args)
		self.name = name
		self.timer = timer
		self.finished = False

		self.cmd_vel_pub = rospy.Publisher('cmd_vel',Twist, queue_size = 10)
		self.cmd_vel_msg = Twist()
		self.cmd_vel_msg.linear.x = 0
		self.cmd_vel_msg.angular.z = 1.5


	def run(self):
		if self.finished:
			return TaskStatus.SUCCESS
		else:

			self.cmd_vel_pub.publish(self.cmd_vel_msg)
			self.timer -= 1
			rospy.sleep(0.1)
			if(self.timer == 0):
				self.finished = True
				self.cmd_vel_pub.publish(Twist())
			return TaskStatus.RUNNING
			


class putDownTask(Task):
	"""docstring for pickUpTask"""
	def __init__(self, name, timer, *args):
		super(putDownTask, self).__init__(name, children = None, *args)
		self.name = name
		self.timer = timer
		self.finished = False

		self.cmd_vel_pub = rospy.Publisher('cmd_vel',Twist, queue_size = 10)
		self.cmd_vel_msg = Twist()
		self.cmd_vel_msg.linear.x = 0
		self.cmd_vel_msg.angular.z = -1.5


	def run(self):
		if self.finished:
			return TaskStatus.SUCCESS
		else:
			self.cmd_vel_pub.publish(self.cmd_vel_msg)
			self.timer -= 1
			rospy.sleep(0.1)
			if(self.timer == 0):
				self.finished = True
				self.cmd_vel_pub.publish(Twist())
			return TaskStatus.RUNNING


class sleepTask(Task):
	"""docstring for pickUpTask"""
	def __init__(self, name, timer, *args):
		super(sleepTask, self).__init__(name, children = None, *args)
		self.name = name
		self.sleep = timer
		self.finished = False


	def run(self):
		if self.sleep == 0:
			self.finished = True

		if self.finished:
			return TaskStatus.SUCCESS
		else:
			self.sleep -= 1
			rospy.sleep(1)
			return TaskStatus.RUNNING


def check_battery(msg):
		if msg.data is None:
			return TaskStatus.RUNNING
		else:
			if msg.data < global_vars.low_battery_threshold:
				rospy.loginfo("LOW BATTERY - level: " + str(int(msg.data)))
				return TaskStatus.FAILURE
			else:
				return TaskStatus.SUCCESS
    
	
def recharge_cb(result):
	rospy.loginfo("BATTERY CHARGED!")

def update_robot_position(msg):
	global_vars.black_board.setCoords("robot", msg.base_position.pose.position.x, msg.base_position.pose.position.y)


class CheckLocation(Task):
    def __init__(self, place, *args, **kwargs):
        name = "checkLocation"
        super(CheckLocation, self).__init__(name)    
        self.name = name
        self.place = place

    def run(self):
        wp = global_vars.black_board.getCoords(self.place).position
        cp = global_vars.black_board.getCoords("robot").position
        
        distance = sqrt((wp.x - cp.x) * (wp.x - cp.x) +
                        (wp.y - cp.y) * (wp.y - cp.y) +
                        (wp.z - cp.z) * (wp.z - cp.z))
                                
        if distance < 0.15:
            status = TaskStatus.SUCCESS
        else:
            status = TaskStatus.FAILURE
            
        return status


class goToTask(SimpleActionTask):
	def __init__(self, name, coords):
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = 'map'
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose = coords
		super(goToTask, self).__init__(name, "move_base", MoveBaseAction, goal, reset_after=False, feedback_cb=update_robot_position)

	def run(self):
		return super(goToTask, self).run()


def update_robot_position(msg):
	global_vars.black_board.setCoords("robot", msg.base_position.pose.position.x, msg.base_position.pose.position.y)





	"""
	Tree generator
	"""


def makeRutines():
	rutines = Sequence("routines")
	for i in range(len(global_vars.black_board.routinesList)):
		rutines.add_child(global_vars.black_board.routinesList[i])

	return rutines

		
def makeTree(plan):

	tree = Sequence("Tree")

	tree.add_child(makeRutines())

	execPlan = Sequence("execPlan")

	lastPlace = "storehouse"

	for i in range(len(plan)):
		if plan[i][0] == global_vars.black_board.movementTask:
			coord = global_vars.black_board.getCoords(plan[i][1])
			execPlan.add_child(goToTask("MoveToTask: " + plan[i][1], coord))
			lastPlace = plan[i][1]
			
		else:
			task = global_vars.black_board.getTask(plan[i][0])
			if task != False:
					
				coords = global_vars.black_board.getCoords(lastPlace)
				moveToLasPositionTask = goToTask("MoveToTaskLastPosition: " + lastPlace, coords)
				checkLocationTask = CheckLocation(lastPlace)

				NavigationTask = Selector("NavRoutine", [checkLocationTask, moveToLasPositionTask])

				execPlan.add_child(Sequence("Task "+ plan[i][0], [NavigationTask, task]))
		
	tree.add_child(execPlan)


	return tree
