#!/usr/bin/env python

"""
global_vars.py - Version 1.0 2015-04-14

Collection of variables used in the application

Copyright (c) 2015 Jose Angel Segura Muros.  All rights reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

"""
import actionlib
import rospy

from geometry_msgs.msg import Twist

from tasks.black_board import *

def init():
	""" Creates the variables
	
	"""
	global black_board 
	black_board = BlackBoard()

	global low_battery_threshold 
	low_battery_threshold = 50.0

	global move_base
	move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

	global cmd_vel_pub
	cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)