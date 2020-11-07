#!/usr/bin/env python

# Import what we need.
import rospy
import numpy as np
import actionlib

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header

from rob599_hw2.srv import memorize_position, memorize_positionResponse
from rob599_hw2.msg import patrolAction, patrolGoal, patrolFeedback, patrolResult

class Memorize:
	def __init__(self):
		# Initialize Action Server
		self.action_server = actionlib.SimpleActionServer('patrol_Action', patrolAction, self.action_callback, False)
		self.action_server.start()

		# Initialize Service Server
		self.service_server = rospy.Service('memorize_position', Memorize_position, self.srv_callback)

		# initialize subscriber
		# self.laser_sub = rospy.Subscriber('filtered_scan', LaserScan, self.laser_callback)

		# initialize publisher
		# self.twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)



	def srv_callback(self,request):

		if self.action_server.is_active():
			rospy.logwarn('sevice server disabled')
			return Stopping_distanceResponse(False)

		else:
			# if request.distance < 1:
			# 	rospy.logwarn('Stopping distance is too short')
			# 	return Stopping_distanceResponse(False)
			#
			# elif request.distance >= 1 and request.distance < self.max_range:
			# 	self.limit = request.distance
			# 	return Stopping_distanceResponse(True)
			# else:
			# 	return Stopping_distanceResponse(False)
			return 0

	def action_callback(self,goal):
		return 0
		# if goal.distance < 1:
		# 	self.action_server.set_succeeded(StopResult(successful=False))
		# 	rospy.logerr('Action failed. Goal distance is under 1 meter')
		# 	return
		#
		# else:
		# 	self.limit = goal.distance
		#
		#
		# while True:
		# 	if self.closest_range < (goal.distance + .05) and self.closest_range > (goal.distance - .05):
		# 		self.action_server.set_succeeded(StopResult(successful=True))
		# 		rospy.loginfo("Action is completed")
		# 		break
		# 	self.action_server.publish_feedback(StopFeedback(progress=self.closest_range - goal.distance))


if __name__ == '__main__':
	# Initialize the node
	rospy.init_node('approach')
	Approach()


	rospy.spin()
