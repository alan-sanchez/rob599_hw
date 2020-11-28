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


	def laser_callback(self, msg, sigma=0.01):
		# Create a LaserScan message named "scan".
		scan = msg

		self.median_filter_pub.publish(scan)


	def generate_sensor_data(n, sigma=0.01):
	    """
	    Generate data from a fake sensor.  These data are generated according to a sine function, with zero-mean
	    Gaussian noise, with standard deviation sigma.
	    """
	    for i in range(n):
	        yield sin(i * 0.01) + gauss(0.0, sigma)


if __name__ == '__main__':
	# Initialize the node
 	rospy.init_node('median_filter')
	Median_filter()

	# Keep running node
	rospy.spin()
