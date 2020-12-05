#!/usr/bin/env python

import rospy
import actionlib
import sys

from geometry_msgs.msg import PolygonStamped, PoseStamped
from std_msgs.msg import Header

# We need the MoveBaseAction and MoveBaseGoal from the move_base_msgs package.
from rob599_hw1.msg import StopAction
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


if __name__ == '__main__':
	# start client
	rospy.init_node('client')
	# Set up a client for the basic nav, receives the user_defined Action
	coverage_client = actionlib.SimpleActionClient('/', PlanMowingPathAction)
	rospy.loginfo('waiting to connect to server...')

	coverage_client.wait_for_server()

	rospy.loginfo('Made contact with move server')
