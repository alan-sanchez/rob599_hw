#!/usr/bin/env python

# Import what we need.
import rospy
import numpy as np
import actionlib

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header

from rob599_hw1.srv import Stopping_distance, Stopping_distanceResponse
from rob599_hw1.msg import StopAction, StopGoal, StopFeedback, StopResult

class Approach:
	def __init__(self):
		# Set up the action server.  The action is called "fetch_status", and has type StopAction.  The
		# function "callback" is called on new requests.  The final argument tells the action client back
		# end not to start autmatically.  You should always set this to False, since if you do otherwise,
		# then you risk race conditions (since the action server back end spins up new threads).
		self.action_server = actionlib.SimpleActionServer('Stop_Action', StopAction, self.action_callback, False)
		self.action_server.start()

		

if __name__ == '__main__':
	# Initialize the node
	rospy.init_node('approach')
	Approach()


	rospy.spin()
