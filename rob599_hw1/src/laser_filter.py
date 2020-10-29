#!/usr/bin/env python

# Import what we need.
import rospy
import numpy as np
from math import pi, atan, sin, cos
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Pose
from visualization_msgs.msg import Marker

class Laser_filter:
	def __init__(self):
		# Set up a subscriber the subscribes to a topic called "base_scan" with
		# a message type LaserScan. This will be sent to the laser_callback function.
		self.laser_sub = rospy.Subscriber('/base_scan', LaserScan, self.laser_callback)

		# Initialize publishers
		self.laser_filter_pub = rospy.Publisher('/filtered_scan',LaserScan, queue_size=10)
		self.line_marker_pub  = rospy.Publisher('line_marker',	 Marker, 	queue_size=10)
		self.text_marker_pub  = rospy.Publisher('text_marker',	 Marker,	queue_size=10)


	# laser_callback called every time we get a LaserScan message from ROS.
	def laser_callback(self, msg):
		# Create a LaserScan message named "scan".
		scan = LaserScan()

		# Becuase the header, angle increment, time increment, minimum range, and
		# maximum range will be the same, we just set those same values for the
		# message definitions of "scan"
		scan.header 		= msg.header
		scan.angle_increment= msg.angle_increment
		scan.time_increment = msg.time_increment
		scan.range_min 		= msg.range_min
		scan.range_max 		= msg.range_max

		# We need to determine what the angle_min is for our filtered "scan".
		# Thus, we define it as None for the time being.
		scan.angle_max = None

		# The for loop is meant to convert the data from the Laserscan messages
		# from polar to cartesian.
		for i in range(len(msg.ranges)):
			# Update both the new theta (relative to the 0 degree mark of the fetch) and
			# and r (range measurement from LaserScan data).
			theta = msg.angle_max - i*msg.angle_increment
			r = msg.ranges[i]

			# We only need to solve the position in the y direction of the robot.
			y_pos = r * sin(theta)

			# A conditional statement to determine if the y_pos is within the 1m width
			# If so, then append that value to the scan.ranges.
			if y_pos < .5 and y_pos > -.5:
					scan.ranges.append(r)

					# The conditional statment below is what sets the scan.angle_min
					# when the FIRST range is appended.
					if scan.angle_max == None:
						scan.angle_max = theta


		# A quick computation that solves the scan.angle_max value
		scan.angle_min = scan.angle_max - (len(scan.ranges)*scan.angle_increment)

		# Create scan.intensities list with all of the elements set to zero.
		scan.intensities = [0] * len(scan.ranges)

		# Publish the filtered "scan" message using the global Publisher.
		self.laser_filter_pub.publish(scan)

		self.line_marker(scan)

	def line_marker(self,scan):
		line = Marker()
		line.header	 = scan.header
		line.type 	 = Marker.LINE_STRIP
		line.action  = Marker.ADD
		line.id 	 = 1
		line.scale.x = 0.03
		line.scale.y = 0.03
		line.color.a = 1
		line.color.r = 1
		line.color.g = 1
		line.color.b = 0

		closest_range = min(scan.ranges)
		index_closest_range = scan.ranges.index(closest_range)

		theta = (index_closest_range*scan.angle_increment) + scan.angle_min

		x_pos = closest_range * cos(theta)
		y_pos = closest_range * sin(theta)

		line.points = [Point(0,0,0), Point(x_pos, y_pos, 0)]

		self.line_marker_pub.publish(line)

		self.text_marker(scan, x_pos, y_pos)

	def text_marker(self,scan, x_pos, y_pos):
		text = Marker()
		text.header	 = scan.header
		text.type 	 = Marker.TEXT_VIEW_FACING
		text.action  = Marker.ADD
		text.id 	 = 1
		text.text 	 = '{:2.1f} m'.format(min(scan.ranges))
		text.scale.z = 0.3
		text.color.a = 1
		text.color.r = 1
		text.color.g = 1
		text.color.b = 1
		text.pose.position.x = x_pos
		text.pose.position.y = y_pos
		text.pose.position.z = -.6

		self.text_marker_pub.publish(text)

if __name__ == '__main__':
	# Initialize the node
 	rospy.init_node('laser_filter')
	Laser_filter()

	# Keep running node
	rospy.spin()
