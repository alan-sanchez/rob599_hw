#!/usr/bin/env python

# Import what we need.
import rospy, actionlib, tf, math, pickle,time
import numpy as np

from geometry_msgs.msg import Twist, Quaternion
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

from rob599_hw2.srv import memorize_position, memorize_positionResponse
from rob599_hw2.srv import read_write, read_writeResponse
from rob599_hw2.msg import goAction, goGoal, goFeedback, goResult
from rob599_hw2.msg import patrolAction, patrolAction, patrolFeedback, patrolResult

class Memorize:
	def __init__(self):
		# Initialize and start Action Servers
		self.go_action_server 		= actionlib.SimpleActionServer('go_Action', goAction, self.go_action_callback, False)
		self.patrol_action_server 	= actionlib.SimpleActionServer('patrol_Action', patrolAction, self.patrol_action_callback, False)
		self.go_action_server.start()
		self.patrol_action_server.start()

		# Initialize Service Server
		self.service_server 	= rospy.Service('memorize_position', memorize_position, self.srv_callback)
		self.rw_service_server 	= rospy.Service('read_write',read_write, self.rw_srv_callback)

		# Make an action client, and wait for the server.
		self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		self.move_base.wait_for_server()
		rospy.loginfo('{0}: Made contact with move_base server'.format(self.__class__.__name__))

		# Set up some dummy callbacks.  These can be overwritten to actually set up callbacks.
		self.active_callback = None
		self.feedback_callback = None
		self.done_callback = None

		# Intialize TransformListner
		self.listener = tf.TransformListener()

		# Intialize Dictionary with start pose and orientation
		self.struct = {"location_1":[[1.0,0.0,0.0],[0,0,0,1]]}

		# Make a goal template, and fill in all of the relevant fields.
		self.goal = MoveBaseGoal()
		self.goal.target_pose.header.frame_id = 'map'
		self.goal.target_pose.header.stamp = rospy.Time()

		# Set a position in the coordinate frame.
		self.goal.target_pose.pose.position.x = 0.0
		self.goal.target_pose.pose.position.y = 0.0
		self.goal.target_pose.pose.position.z = 0.0

		# Set the orientation in the coordinate frame.
		self.goal.target_pose.pose.orientation = Quaternion(0,0,0,1)

		# Set rate
		self.r = rospy.Rate(.5)

	# Service callback for stored location
	def srv_callback(self,request):
		# Get position and quaternion of current location.
		position,quaternion = self.get_location(request.location)

		# Add location name and values in data structure
		self.struct[request.location] =[position,quaternion]
		return memorize_positionResponse("Location at {0} is stored.".format(position))


	# Service callback for creating and uploading data structure of locations
	def rw_srv_callback(self,request):
		# Store data in locations.pickle file
		if request.order == "write":
			with open('locations.pickle', 'wb') as handle:
				pickle.dump(self.struct, handle, protocol=None)
			return read_writeResponse("The data structure has been saved")

		# Load data from locations.pickle file
		elif request.order == "read":
			with open('locations.pickle', 'rb') as handle:
				self.struct = pickle.load(handle)
				print(self.struct)
			return read_writeResponse("The data structure has been uploaded:")

		else:
			return read_writeResponse("You need to enter either 'read' or 'write'")


	# Action server to move the fetch to desried location
	def go_action_callback(self,goal):

		if goal.location in self.struct:
			result  = self.move_to(goal.location,"go")
			self.go_action_server.set_succeeded(goResult(successful=result))
		else:
			rospy.logerr("You typed in a location that doesn't exist in the data structure.")
			self.go_action_server.set_succeeded(goResult(successful=False))

	# Action sever to run patrol
	def patrol_action_callback(self,goal):
		# Forloop for patrol movement
		for key in self.struct:
			bool = self.move_to(key,"patrol")
			rospy.loginfo("Fetch reached {0}: {1}".format(key, bool))
			self.r.sleep()

		self.patrol_action_server.set_succeeded(patrolResult(successful=True))



	# Function that aquires pose of location
  	def get_location(self, location):
		# Begin whilelopp that will perform the lookupTransform between the map and base_link
		while not rospy.is_shutdown():
			try:
				position,quaternion = self.listener.lookupTransform("/map","/base_link", rospy.Time(0))
				return position,quaternion
				break

			except (tf.LookupException, tf.ConnectivityException,tf.ExtrapolationException):
				pass


	def move_to(self, location, act):
		# Set the x and y positions
		self.goal.target_pose.pose.position.x = self.struct[location][0][0]
		self.goal.target_pose.pose.position.y = self.struct[location][0][1]

		# Set the orientation.
		self.goal.target_pose.pose.orientation = Quaternion(*self.struct[location][1])

		# Make the action call.  Include the callbacks.  Unless these have been set somewhere else, they are passed
		# as None, which means no callback.
		self.move_base.send_goal(self.goal, active_cb=self.active_callback, feedback_cb=self.feedback_callback,
			done_cb=self.done_callback)

		# Euclidean distance feedback
		time_out = 0
		if act == "go":
			while self.move_base.get_state() != GoalStatus.SUCCEEDED and time_out < 40:
				position,_ = self.get_location(location)
				x = position[0] - self.goal.target_pose.pose.position.x
				y = position[1] - self.goal.target_pose.pose.position.y
				eucl_dist = math.hypot(x,y)
				self.go_action_server.publish_feedback(goFeedback(progress=str(eucl_dist)))
				#After 20 seconds the loop will break
				time_out += 1
				time.sleep(.5)
		else:
			while self.move_base.get_state() != GoalStatus.SUCCEEDED and time_out < 40:
				position,_ = self.get_location(location)
				x = position[0] - self.goal.target_pose.pose.position.x
				y = position[1] - self.goal.target_pose.pose.position.y
				eucl_dist = math.hypot(x,y)
				self.patrol_action_server.publish_feedback(patrolFeedback(progress=str(eucl_dist)))
				#After 20 seconds the loop will break
				time_out += 1
				time.sleep(.5)

		# Wait for result and return a boolean of the success
		self.move_base.wait_for_result()
		return self.move_base.get_state() == GoalStatus.SUCCEEDED


if __name__ == '__main__':
	# Initialize the node
	rospy.init_node('memorize')
	Memorize()

	rospy.spin()
