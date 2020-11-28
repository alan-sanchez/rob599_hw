#!/usr/bin/env python

# Import what we need.
import rospy
import numpy as np
from math import pi, atan, sin, cos
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Pose
from visualization_msgs.msg import Marker

class Median_filter:
	def __init__(self):
		# Initialize subscriber
		self.laser_sub = rospy.Subscriber('/base_scan', LaserScan, self.laser_callback)

		# Initialize publishers
		self.median_filter_pub = rospy.Publisher('/median_filter', LaserScan, queue_size=1)


	def laser_callback(self, msg):
		# Create a LaserScan message named "scan".
		scan = msg

		self.median_filter_pub.publish(scan)

if __name__ == '__main__':
	# Initialize the node
 	rospy.init_node('median_filter')
	Median_filter()

	# Keep running node
	rospy.spin()
