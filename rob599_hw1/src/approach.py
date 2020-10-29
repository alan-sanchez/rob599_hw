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

		# Create the service, specifying the name, the service message base type, and the callback function
		# to be used to fulfil the requests.
		self.service_server = rospy.Service('stopping_distance', Stopping_distance, self.srv_callback)

		# Set up a subscriber the subscribes to a topic called "filtered_scan" with
		# a message type LaserScan and will be sent to the laser_callback function.
		# Also,set up a publisher that will send the velocity commands for the fetch.
		self.laser_sub = rospy.Subscriber('filtered_scan', LaserScan, self.laser_callback)
		self.twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

		#initalizing the limit to be 1 meter
		self.limit = 1
		self.max_range = 25
		self.closest_range = 0


	# laser_callback called every time we get a LaserScan message from ROS.
	def laser_callback(self, msg):
		# Below the closest range is stored.
		# Speed regulation will be a hyperbolic tangent function
		self.closest_range=min(msg.ranges)
		speed_reg = np.tanh(self.closest_range-self.limit)

		# Set the velocity for the linear x direction of the robot
		command = Twist()
		if self.closest_range < (self.limit + .05) and self.closest_range > (self.limit - .05):
			command.linear.x = 0
		else:
			command.linear.x = speed_reg

		# Publish the command using the global Publisher
		self.twist_pub.publish(command)


	def srv_callback(self,request):

		if self.action_server.is_active():
			rospy.logwarn('sevice server disabled')
			return Stopping_distanceResponse(False)

		else:
			if request.distance < 1:
				rospy.logwarn('Stopping distance is too short')
				return Stopping_distanceResponse(False)

			elif request.distance >= 1 and request.distance < self.max_range:
				self.limit = request.distance
				return Stopping_distanceResponse(True)
			else:
				return Stopping_distanceResponse(False)


	def action_callback(self,goal):

		if goal.distance < 1:
			self.action_server.set_succeeded(StopResult(successful=False))
			rospy.logerr('Action failed. Goal distance is under 1 meter')
			return

		else:
			self.limit = goal.distance


		while True:
			if self.closest_range < (goal.distance + .05) and self.closest_range > (goal.distance - .05):
				self.action_server.set_succeeded(StopResult(successful=True))
				rospy.loginfo("Action is completed")
				break
			self.action_server.publish_feedback(StopFeedback(progress=self.closest_range - goal.distance))


if __name__ == '__main__':
	# Initialize the node
	rospy.init_node('approach')
	Approach()


	rospy.spin()
